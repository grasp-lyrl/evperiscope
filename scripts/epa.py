#!/usr/bin/env python3
"""Offline event-based propeller pose estimation pipeline.

Runs the same detection and pose-estimation stages as the ROS 2 node
``epa::EpaController`` (``src/epa_controller.cpp``) over a recorded HDF5 event
file, and writes an annotated video. The flight-control half of the C++ node
has no counterpart here.

Example:
    python3 epa.py events.h5 --calib ../config/calib_epa.xml -o output.mp4
"""

import argparse
import time

import cv2
import h5py
import numpy as np

from detect import EvPropDet
from epa_filter import QuadPoseFilter


def load_calibration(filepath):
    """Read intrinsics from an OpenCV FileStorage XML calibration file.

    The camera node is found by looking for the first root child that carries a
    ``camera_matrix``, so the node name (e.g. the serial number) does not matter.

    Args:
        filepath: Path to the calibration XML.

    Returns:
        Tuple of (K, dist_coeffs, width, height).

    Raises:
        RuntimeError: If the file cannot be opened or holds no camera node.
    """
    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f'Cannot open calibration file: {filepath}')

    try:
        for key in fs.root().keys():
            node = fs.root().getNode(key)
            if node.getNode('camera_matrix').empty():
                continue
            K = node.getNode('camera_matrix').mat()
            dist = node.getNode('distortion_coefficients').mat()
            width = int(node.getNode('image_width').real())
            height = int(node.getNode('image_height').real())
            return K, dist, width, height
    finally:
        fs.release()

    raise RuntimeError(f'No camera node in calibration file: {filepath}')


def build_undistort_map(K, dist_coeffs, width, height):
    """Precompute where every raw pixel lands once undistorted.

    Events arrive as individual pixel coordinates rather than an image, so
    remapping them one by one through a lookup table is cheaper than
    undistorting a frame.

    Args:
        K: 3x3 raw camera matrix.
        dist_coeffs: Distortion coefficients.
        width, height: Sensor resolution.

    Returns:
        Tuple of (map_x, map_y, K_rect), where the maps are (height, width)
        float32 arrays and K_rect is the rectified camera matrix.
    """
    # alpha=0 crops to the region with no invalid pixels
    K_rect, _ = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, (width, height), 0.0)

    grid_x, grid_y = np.meshgrid(np.arange(width), np.arange(height))
    pixels = np.stack([grid_x, grid_y], axis=-1).reshape(-1, 1, 2).astype(np.float32)

    undistorted = cv2.undistortPoints(pixels, K, dist_coeffs, None, K_rect)
    undistorted = undistorted.reshape(height, width, 2)

    return (np.ascontiguousarray(undistorted[..., 0]),
            np.ascontiguousarray(undistorted[..., 1]),
            K_rect)


def load_events(filepath, map_x, map_y, width, height):
    """Load an HDF5 event file and undistort the event coordinates.

    Args:
        filepath: Path to the HDF5 file, with an ``events`` group holding the
            ``p``, ``t``, ``x`` and ``y`` datasets.
        map_x, map_y: Undistortion lookup tables.
        width, height: Sensor resolution.

    Returns:
        Tuple of (p, t, x, y). Timestamps are seconds relative to the first
        event; coordinates are undistorted integer pixels.
    """
    with h5py.File(filepath, 'r') as data:
        events = data['events']
        p = events['p'][:].astype(np.float32)
        t = events['t'][:]
        x = events['x'][:].astype(np.int64)
        y = events['y'][:].astype(np.int64)

    t = (t - t[0]) / 1e6  # microseconds since the first event -> seconds

    in_bounds = (x >= 0) & (x < width) & (y >= 0) & (y < height)
    ux, uy = x.copy(), y.copy()
    ux[in_bounds] = np.round(map_x[y[in_bounds], x[in_bounds]]).astype(np.int64)
    uy[in_bounds] = np.round(map_y[y[in_bounds], x[in_bounds]]).astype(np.int64)

    return p, t, ux, uy


def get_event_slice(ps, ts, xs, ys, t_start, t_end):
    """Return the events with timestamps in [t_start, t_end)."""
    start, end = np.searchsorted(ts, [t_start, t_end])
    return ps[start:end], ts[start:end], xs[start:end], ys[start:end]


def measured_dt(t_end, last_t, nominal):
    """Interval since the previous update, clamped to a plausible range.

    The EKF needs the interval that actually elapsed rather than the nominal
    batch duration, since detections are only fed to it on the frames where
    they are stable. Mirrors ``EpaController::measuredDt``.

    Args:
        t_end: End of the current batch, in seconds.
        last_t: End of the previous update's batch, or None on the first call.
        nominal: Nominal batch duration, used as the fallback and clamp scale.

    Returns:
        Tuple of (dt, new_last_t).
    """
    if last_t is None:
        return nominal, t_end
    return float(np.clip(t_end - last_t, 0.1 * nominal, 10.0 * nominal)), t_end


def annotate(image, pose_filter, pose, t_end, detect_ms, pose_ms, height):
    """Draw the reprojected model, pose readout and timings onto the frame."""
    projected = pose_filter.get_projected_centroids()
    if projected is not None:
        for p in projected.astype(np.int32):
            cv2.circle(image, (p[1], p[0]), 8, (0, 255, 0), 2)

    if pose is not None:
        x, y, z, roll, pitch, yaw = pose
        cv2.putText(image, f'Pos: {x:.2f} {y:.2f} {z:.2f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(image, f'Att: {np.degrees(roll):.1f} {np.degrees(pitch):.1f} '
                           f'{np.degrees(yaw):.1f}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        axes_pts = pose_filter.get_axes(length=0.2)
        if axes_pts is not None:
            origin, x_tip, _, _ = axes_pts
            cv2.arrowedLine(image, (origin[1], origin[0]), (x_tip[1], x_tip[0]),
                            (0, 0, 255), 3, tipLength=0.2)

    cv2.putText(image, f't={t_end:.3f}s | det:{detect_ms:.1f}ms | pose:{pose_ms:.1f}ms',
                (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('events', help='HDF5 event file')
    parser.add_argument('--calib', required=True,
                        help='OpenCV FileStorage XML camera calibration')
    parser.add_argument('-o', '--output', default='output.mp4',
                        help='Annotated video to write (default: output.mp4)')
    parser.add_argument('--tau', type=float, default=0.02,
                        help='Exponential filter time constant, seconds')
    parser.add_argument('--duration', type=float, default=0.005,
                        help='Event batch duration, seconds')
    parser.add_argument('--t-start', type=float, default=15.0,
                        help='Start time within the recording, seconds')
    parser.add_argument('--max-frames', type=int, default=700,
                        help='Stop after this many batches (0 for no limit)')
    parser.add_argument('--n-centroids', type=int, default=4,
                        help='Number of propeller centroids to track')
    parser.add_argument('--front-arm-length', type=float, default=0.13125)
    parser.add_argument('--rear-arm-length', type=float, default=0.11625)
    parser.add_argument('--front-back-offset', type=float, default=0.09625)
    return parser.parse_args()


def main():
    args = parse_args()

    K, dist_coeffs, width, height = load_calibration(args.calib)
    map_x, map_y, K_rect = build_undistort_map(K, dist_coeffs, width, height)
    print(f'Calibration {width}x{height}, rectified fx={K_rect[0, 0]:.2f} '
          f'fy={K_rect[1, 1]:.2f} cx={K_rect[0, 2]:.2f} cy={K_rect[1, 2]:.2f}')

    ps, ts, xs, ys = load_events(args.events, map_x, map_y, width, height)
    print(f'Loaded {len(ts)} events spanning {ts[-1]:.3f} seconds')

    detector = EvPropDet(h=height, w=width, tau=args.tau,
                         n_centroids=args.n_centroids)
    pose_filter = QuadPoseFilter(
        camera_matrix=K_rect,
        front_arm_length=args.front_arm_length,
        rear_arm_length=args.rear_arm_length,
        front_back_offset=args.front_back_offset,
    )

    fps = round(1.0 / args.duration)
    video_out = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'),
                                fps, (width, height))

    t_start = args.t_start
    t_end = t_start + args.duration
    frame_count = 0
    last_pose_update_t = None

    try:
        while t_end <= ts[-1]:
            if args.max_frames and frame_count >= args.max_frames:
                break

            p_slice, t_slice, x_slice, y_slice = get_event_slice(
                ps, ts, xs, ys, t_start, t_end)

            if len(x_slice) > 0:
                start_time = time.time()
                result = detector.process_frame(
                    p_slice, t_slice, x_slice, y_slice, t_end)
                detect_ms = (time.time() - start_time) * 1000

                pose_ms = 0.0
                pose = None
                if result['is_stable'] and result['centroids'] is not None:
                    dt, last_pose_update_t = measured_dt(
                        t_end, last_pose_update_t, args.duration)
                    filter_start = time.time()
                    pose_filter.update(result['centroids'], dt=dt)
                    pose_ms = (time.time() - filter_start) * 1000
                    pose = pose_filter.get_pose()

                annotate(result['image'], pose_filter, pose, t_end,
                         detect_ms, pose_ms, height)
                video_out.write(result['image'])

                if frame_count % 50 == 0:
                    print(f'Frame {frame_count}: t={t_end:.3f}s, '
                          f'detect={detect_ms:.2f}ms, pose={pose_ms:.2f}ms, '
                          f"stable={result['is_stable']}")

            t_start += args.duration
            t_end += args.duration
            frame_count += 1
    finally:
        video_out.release()

    print(f'\nProcessed {frame_count} frames, saved to {args.output}')


if __name__ == '__main__':
    main()
