"""Quadrotor 6DoF pose filter.

Reference Python implementation of ``epa::QuadPoseFilter``
(``src/quad_pose_filter.cpp``). Both implementations are kept numerically
equivalent.

An extended Kalman filter with a constant-velocity motion model estimates the
pose of an asymmetric quadrotor from four propeller centroid observations.

    State:       [x, y, z, roll, pitch, yaw, vx, vy, vz, v_roll, v_pitch, v_yaw]
    Measurement: four centroids in (row, col) pixel coordinates, 8 values

Pixel coordinates are (row, col) throughout the public interface. OpenCV wants
(x, y) = (col, row), so conversions happen at the OpenCV call sites only.
"""

import numpy as np
import cv2

from itertools import permutations

# Rejection limits applied when seeding the filter from a single PnP solve
MAX_INIT_ROLL_PITCH_RAD = np.deg2rad(20.0)
MIN_INIT_ALTITUDE_M = 0.0
MAX_INIT_ALTITUDE_M = 5.0


def _euler_to_rotation_matrix(roll, pitch, yaw):
    """Build the ZYX rotation matrix Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=np.float64)


def _compute_derivatives_rotation_matrix(roll, pitch, yaw, points_3d):
    """Differentiate ``R @ p`` with respect to roll, pitch and yaw.

    Args:
        roll, pitch, yaw: Current attitude in radians.
        points_3d: (N, 3) body-frame points.

    Returns:
        Three (N, 3) arrays: d(R@p)/droll, d(R@p)/dpitch, d(R@p)/dyaw.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # dR/droll = Rz(yaw) @ Ry(pitch) @ dRx(roll)/droll
    dR_dr = np.array([
        [0.0, cy * sp * cr + sy * sr, -cy * sp * sr + sy * cr],
        [0.0, sy * sp * cr - cy * sr, -sy * sp * sr - cy * cr],
        [0.0, cp * cr,                -cp * sr],
    ], dtype=np.float64)

    # dR/dpitch = Rz(yaw) @ dRy(pitch)/dpitch @ Rx(roll)
    dR_dp = np.array([
        [-cy * sp, cy * cp * sr, cy * cp * cr],
        [-sy * sp, sy * cp * sr, sy * cp * cr],
        [-cp,      -sp * sr,     -sp * cr],
    ], dtype=np.float64)

    # dR/dyaw = dRz(yaw)/dyaw @ Ry(pitch) @ Rx(roll)
    dR_dy = np.array([
        [-sy * cp, -sy * sp * sr - cy * cr, -sy * sp * cr + cy * sr],
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
        [0.0,      0.0,                     0.0],
    ], dtype=np.float64)

    points_3d = np.asarray(points_3d, dtype=np.float64)
    return (points_3d @ dR_dr.T, points_3d @ dR_dp.T, points_3d @ dR_dy.T)


def _project_points(points_3d, rvec, tvec, fx, fy, cx, cy):
    """Pinhole-project body-frame points.

    Args:
        points_3d: (N, 3) body-frame points.
        rvec: Attitude as (roll, pitch, yaw) in radians.
        tvec: Translation from body to camera frame, in metres.
        fx, fy, cx, cy: Camera intrinsics.

    Returns:
        (N, 2) array of (u, v) = (col, row) pixels. Points at or behind the
        camera are returned as (-1, -1).
    """
    R = _euler_to_rotation_matrix(rvec[0], rvec[1], rvec[2])
    p_cam = np.asarray(points_3d, dtype=np.float64) @ R.T + np.asarray(tvec, dtype=np.float64)

    points_2d = np.full((len(p_cam), 2), -1.0, dtype=np.float64)
    visible = p_cam[:, 2] > 0
    z = p_cam[visible, 2]
    points_2d[visible, 0] = fx * p_cam[visible, 0] / z + cx
    points_2d[visible, 1] = fy * p_cam[visible, 1] / z + cy
    return points_2d


def _compute_jacobian_6dof(points_3d, state6, fx, fy):
    """Jacobian of the projected centroids w.r.t. [x, y, z, roll, pitch, yaw].

    Args:
        points_3d: (N, 3) body-frame model points.
        state6: The pose part of the filter state.
        fx, fy: Focal lengths in pixels.

    Returns:
        (2N, 6) Jacobian, rows interleaved as (u, v) per point.
    """
    x, y, z, roll, pitch, yaw = state6
    tvec = np.array([x, y, z], dtype=np.float64)

    R = _euler_to_rotation_matrix(roll, pitch, yaw)
    dP_droll, dP_dpitch, dP_dyaw = _compute_derivatives_rotation_matrix(
        roll, pitch, yaw, points_3d)

    n_points = len(points_3d)
    J = np.zeros((2 * n_points, 6), dtype=np.float64)

    for i in range(n_points):
        X, Y, Z = np.asarray(points_3d[i], dtype=np.float64) @ R.T + tvec
        if Z <= 0.1:
            continue

        # d(u, v) / d(X, Y, Z); du/dY and dv/dX are identically zero
        du_dX = fx / Z
        du_dZ = -fx * X / (Z * Z)
        dv_dY = fy / Z
        dv_dZ = -fy * Y / (Z * Z)

        # Translation: dP_cam/dx = [1, 0, 0], and so on
        J[2 * i, 0] = du_dX
        J[2 * i, 2] = du_dZ
        J[2 * i + 1, 1] = dv_dY
        J[2 * i + 1, 2] = dv_dZ

        # Attitude, by the chain rule d(u,v)/dtheta = d(u,v)/dP @ dP/dtheta
        for col, dP in enumerate((dP_droll, dP_dpitch, dP_dyaw), start=3):
            J[2 * i, col] = du_dX * dP[i, 0] + du_dZ * dP[i, 2]
            J[2 * i + 1, col] = dv_dY * dP[i, 1] + dv_dZ * dP[i, 2]

    return J


class QuadPoseFilter:
    """Constant-velocity EKF tracking a quadrotor from propeller centroids."""

    def __init__(self, camera_matrix, front_arm_length=0.15, rear_arm_length=0.12,
                 front_back_offset=0.1, use_brute_force_ordering=False):
        """
        Args:
            camera_matrix: 3x3 intrinsics. Use the *rectified* intrinsics if the
                centroids come from undistorted event coordinates.
            front_arm_length: Front arm half-span, in metres.
            rear_arm_length: Rear arm half-span, in metres.
            front_back_offset: Longitudinal offset of each arm from the centre.
            use_brute_force_ordering: Order centroids by trying all 24
                permutations and keeping the lowest reprojection error, instead
                of the geometric heuristic. Slower but more robust.
        """
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

        # Model points in the body frame (X forward, Y left, Z up)
        self.model_points = np.array([
            [front_back_offset, front_arm_length, 0.0],    # FL
            [front_back_offset, -front_arm_length, 0.0],   # FR
            [-front_back_offset, -rear_arm_length, 0.0],   # RR
            [-front_back_offset, rear_arm_length, 0.0],    # RL
        ], dtype=np.float64)

        self.state = None
        self.P = None
        self.initialized = False
        self.use_brute_force_ordering = use_brute_force_ordering

        # Process noise: pose drifts slowly, velocities are far less certain
        self.Q = np.eye(12) * 0.001
        self.Q[6:, 6:] *= 10.0

        # Measurement noise, in pixels squared
        self.R = np.diag([5.0, 5.0] * 4)

    # ---------- Centroid ordering ----------

    def _order_centroids_brute_force(self, centroids):
        """Order centroids by minimising PnP reprojection error over all 24
        permutations.

        Args:
            centroids: (4, 2) array of (row, col) centroids.

        Returns:
            (4, 2) array of (row, col) centroids ordered [FL, FR, RR, RL],
            or None if no permutation yielded a valid pose.
        """
        centroids = np.asarray(centroids, dtype=np.float64)
        best_error = float('inf')
        best_ordered = None

        for perm in permutations(range(4)):
            ordered = centroids[list(perm)]
            image_points = np.column_stack([ordered[:, 1], ordered[:, 0]])

            success, rvec, tvec = cv2.solvePnP(
                self.model_points, image_points, self.camera_matrix, None,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not success or tvec[2, 0] < 0.1:
                continue

            reproj, _ = cv2.projectPoints(
                self.model_points, rvec, tvec, self.camera_matrix, None)
            error = np.mean(np.linalg.norm(reproj.reshape(-1, 2) - image_points, axis=1))

            if error < best_error:
                best_error = error
                best_ordered = ordered

        return best_ordered

    def _order_centroids_by_distance(self, centroids):
        """Order centroids as [FL, FR, RR, RL] using a geometric heuristic.

        The quadrotor is asymmetric front-to-back, so the widest adjacent pair
        in the angularly sorted quad is the front edge. Inserting the resulting
        yaw angle back into the angular ordering fixes where the traversal
        starts, which pins each centroid to a specific arm.

        Args:
            centroids: (4, 2) array of (row, col) centroids.

        Returns:
            (4, 2) array of (row, col) centroids ordered [FL, FR, RR, RL].
        """
        centroids = np.asarray(centroids, dtype=np.float64)

        # Work in image (x, y) = (col, row) so the angles below are conventional
        points = np.column_stack([centroids[:, 1], centroids[:, 0]])
        quad_center = np.mean(points, axis=0)

        # 1. Sort the four points counter-clockwise about their centroid
        angles = np.arctan2(points[:, 1] - quad_center[1], points[:, 0] - quad_center[0])
        angles = (angles + 2 * np.pi) % (2 * np.pi)
        idx = np.argsort(angles)
        sorted_pts = points[idx]

        # 2. The front pair is the adjacent pair with the largest separation
        adj_dists = np.linalg.norm(np.diff(sorted_pts, axis=0), axis=1)
        adj_dists = np.append(adj_dists, np.linalg.norm(sorted_pts[0] - sorted_pts[-1]))
        max_idx = np.argmax(adj_dists)
        front_pts = points[idx[[max_idx, (max_idx + 1) % 4]]]

        # 3. Forward direction runs from the quad centre towards the front pair
        forward_vec = np.mean(front_pts, axis=0) - quad_center
        yaw = np.arctan2(forward_vec[1], forward_vec[0])
        yaw = (yaw + 2 * np.pi) % (2 * np.pi)

        # 4. Start the counter-clockwise traversal just past the forward
        #    direction, so the first point encountered is the front-left arm
        all_angles = np.append(angles, yaw)
        all_sort_idx = np.argsort(all_angles)
        yaw_pos = np.argwhere(all_sort_idx == 4)[0][0]
        ordered_idx = all_sort_idx[np.arange(yaw_pos + 1, yaw_pos + 5) % 5]
        ordered_idx = ordered_idx[ordered_idx < 4]

        if len(ordered_idx) < 4:
            # Degenerate geometry: fall back to the detector's own order
            return centroids

        ordered = points[ordered_idx]

        # The traversal is counter-clockwise from FL; the model order is
        # FL, FR, RR, RL, so the last three are taken in reverse
        ordered_new = np.array([ordered[0], ordered[3], ordered[2], ordered[1]])

        # Back to (row, col)
        return np.column_stack([ordered_new[:, 1], ordered_new[:, 0]])

    def _order_centroids(self, centroids):
        """Dispatch to the configured ordering method. Returns None on failure."""
        if len(centroids) != 4:
            return None
        if self.use_brute_force_ordering:
            return self._order_centroids_brute_force(centroids)
        return self._order_centroids_by_distance(centroids)

    # ---------- EKF ----------

    def initialize(self, centroids):
        """Seed the filter from a single PnP solve.

        The resulting pose is rejected if it is implausible for a quadrotor
        hovering below the camera, which keeps a bad centroid ordering from
        locking the filter onto a spurious pose.

        Args:
            centroids: (4, 2) array of (row, col) centroids.

        Returns:
            True if the filter is now initialised.
        """
        ordered = self._order_centroids(centroids)
        if ordered is None:
            return False

        image_points = np.column_stack([ordered[:, 1], ordered[:, 0]]).astype(np.float64)
        success, rvec, tvec = cv2.solvePnP(
            self.model_points, image_points, self.camera_matrix, None,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            return False

        tvec = tvec.flatten()
        matrix, _ = cv2.Rodrigues(rvec.flatten())

        # Extract ZYX Euler angles from the rotation matrix
        sy = np.sqrt(matrix[0, 0] ** 2 + matrix[1, 0] ** 2)
        if sy > 1e-6:
            roll = np.arctan2(matrix[2, 1], matrix[2, 2])
            pitch = np.arctan2(-matrix[2, 0], sy)
            yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
        else:
            # Gimbal lock: yaw is unobservable, fold it into roll
            roll = np.arctan2(-matrix[1, 2], matrix[1, 1])
            pitch = np.arctan2(-matrix[2, 0], sy)
            yaw = 0.0

        if (abs(roll) > MAX_INIT_ROLL_PITCH_RAD or
                abs(pitch) > MAX_INIT_ROLL_PITCH_RAD):
            return False
        if not (MIN_INIT_ALTITUDE_M <= tvec[2] <= MAX_INIT_ALTITUDE_M):
            return False

        self.state = np.zeros(12, dtype=np.float64)
        self.state[:6] = [tvec[0], tvec[1], tvec[2], roll, pitch, yaw]

        self.P = np.eye(12) * 0.1
        self.P[6:, 6:] = np.eye(6) * 1.0  # velocities start highly uncertain

        self.initialized = True
        return True

    def predict(self, dt):
        """Propagate the constant-velocity model forward by ``dt`` seconds."""
        if not self.initialized:
            return

        F = np.eye(12, dtype=np.float64)
        for i in range(6):
            F[i, i + 6] = dt

        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q

    def update(self, centroids, dt=0.01):
        """Run one predict/update cycle against a new centroid observation.

        Args:
            centroids: (4, 2) array of (row, col) centroids.
            dt: Time since the previous update, in seconds.

        Returns:
            True if the state was updated (or the filter was just initialised).
        """
        if not self.initialized:
            return self.initialize(centroids)

        ordered = self._order_centroids(centroids)
        if ordered is None:
            return False

        self.predict(dt)

        state6 = self.state[:6]
        predicted_pts = _project_points(
            self.model_points, state6[3:6], state6[:3], self.fx, self.fy, self.cx, self.cy)

        # Residual, both sides in (row, col)
        z_meas = np.asarray(ordered, dtype=np.float64).flatten()
        h_meas = np.column_stack([predicted_pts[:, 1], predicted_pts[:, 0]]).flatten()
        y_residual = z_meas - h_meas

        # Measurements do not depend on the velocity states
        H_12 = np.zeros((8, 12), dtype=np.float64)
        H_12[:, :6] = _compute_jacobian_6dof(self.model_points, state6, self.fx, self.fy)

        # The Jacobian rows are (u, v); swap them to match the (row, col) residual
        H_rc = np.empty_like(H_12)
        H_rc[0::2] = H_12[1::2]
        H_rc[1::2] = H_12[0::2]

        S = H_rc @ self.P @ H_rc.T + self.R
        try:
            K = self.P @ H_rc.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False

        self.state = self.state + K @ y_residual

        # Joseph form, which stays symmetric and positive definite
        IKH = np.eye(12) - K @ H_rc
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T

        return True

    # ---------- Accessors ----------

    def get_pose(self):
        """Return (x, y, z, roll, pitch, yaw), or None if not initialised."""
        if not self.initialized:
            return None
        return tuple(self.state[:6])

    def get_twist(self):
        """Return (vx, vy, vz, v_roll, v_pitch, v_yaw), or None."""
        if not self.initialized:
            return None
        return tuple(self.state[6:])

    def get_projected_centroids(self):
        """Return the (4, 2) (row, col) reprojection of the model points."""
        if not self.initialized:
            return None
        state6 = self.state[:6]
        pts = _project_points(self.model_points, state6[3:6], state6[:3],
                              self.fx, self.fy, self.cx, self.cy)
        return np.column_stack([pts[:, 1], pts[:, 0]])

    def get_axes(self, length=0.2):
        """Project the body axes for visualisation.

        Args:
            length: Axis length in metres.

        Returns:
            Four (row, col) integer tuples: origin, X tip, Y tip, Z tip.
        """
        if not self.initialized:
            return None
        state6 = self.state[:6]

        pts_3d = np.array([
            [0.0, 0.0, 0.0],
            [length, 0.0, 0.0],
            [0.0, length, 0.0],
            [0.0, 0.0, length],
        ], dtype=np.float64)

        pts = _project_points(pts_3d, state6[3:6], state6[:3],
                              self.fx, self.fy, self.cx, self.cy)
        return tuple((int(p[1]), int(p[0])) for p in pts)
