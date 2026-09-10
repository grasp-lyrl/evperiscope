<div align="center">

# EVPeriscope

### Extended Perception across Aerial and Ground Vehicles with<br>Event-based Propeller Tracking

[Dexter Ong](https://dexterong.com) &nbsp;&middot;&nbsp; [Vijay Kumar](https://www.kumarrobotics.org) &nbsp;&middot;&nbsp; [Pratik Chaudhari](https://pratikac.github.io)

GRASP Laboratory, University of Pennsylvania

**International Symposium of Robotics Research (ISRR) 2026**

[![Paper](https://img.shields.io/badge/Paper-b31b1b?style=for-the-badge&logo=arxiv&logoColor=white)](https://ongdexter.github.io/evperiscope/static/pdfs/evperiscope.pdf)
[![Project Page](https://img.shields.io/badge/Project%20Page-1a73e8?style=for-the-badge&logo=firefox&logoColor=white)](https://ongdexter.github.io/evperiscope)
[![Video](https://img.shields.io/badge/Video-ff0000?style=for-the-badge&logo=youtube&logoColor=white)](https://drive.google.com/file/d/1HxsL9gdY0HvlqUsuwir866r7UxwZev7N/view?usp=drive_link)

<video src="https://github.com/user-attachments/assets/e1b10645-195e-4a07-9a44-6dcb75b8ef6b"
       width="75%" autoplay muted loop playsinline></video>

</div>

## Overview

Reliable relative localization between aerial and ground robots is a key
requirement for tightly coordinated heterogeneous teams, and is difficult to
achieve with conventional frame-based cameras and fiducial markers because they
are sensitive to motion blur, lighting variations, and payload constraints.

EVPeriscope is an event-based perception system that detects, localizes and
controls a quadrotor using an upward-facing event camera on a ground robot,
keying on the high-frequency visual signature of the spinning propellers rather
than on any added marker. This lets the quadrotor act as an extended perception
system for the ground robot when its own sensors are degraded or occluded.

We demonstrate the marsupial ground–aerial system in field conditions with wind
speeds up to 15 mph, in daylight and at night, and show that it supports
localization and closed-loop navigation through dense foliage where the ground
robot's sensors are occluded. The control system runs at 200 Hz entirely on
onboard sensing and computation.

This repository contains the perception and control stack:

- **`epa::EpaController`** — event detection, pose estimation, tracking and the
  landing state machine
- **`epa::EpaInterface`** — MAVROS bridge (arm / offboard / takeoff / land,
  ZED VIO → PX4 vision pose)

A reference implementation of the same estimator in Python, for offline
analysis, lives in [`scripts/`](scripts/). It reads HDF5 event files, which
`dv_ros2_bag_to_h5.py` in
[`dv-ros2-wrapper`](https://github.com/grasp-lyrl/dv-ros2-wrapper) generates
from a bag.

## Setup

Tested on Ubuntu 22.04 with ROS 2 Humble. See [`docs/setup.md`](docs/setup.md)
for step-by-step instructions and troubleshooting. Prerequisites:

- **GCC 13 or newer** — required by `dv-processing`
- **[dv-processing](https://gitlab.com/inivation/dv/dv-processing) 2.0.3**

```bash
mkdir -p ~/epa_ws/src && cd ~/epa_ws/src

git clone https://github.com/grasp-lyrl/evperiscope.git
git clone https://github.com/grasp-lyrl/dv-ros2-wrapper.git

cd ~/epa_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13
source install/setup.bash
```

`dv-ros2-wrapper` provides the event camera driver and the `EventArray` message
type this package subscribes to.

## Running

A sample bag is available here:
[**epa_sample**](https://drive.google.com/drive/folders/1D0P2mjQJbZRxfLzocB7dgYOWYneFKuPr?usp=sharing).

Then, in three terminals, each with the workspace sourced:

```bash
# 1. the pipeline
ros2 launch epa epa_bag.launch.py

# 2. the bag
ros2 bag play /path/to/epa_sample

# 3. visualization
ros2 run rqt_image_view rqt_image_view /epa/debug_image
```

`/epa/debug_image` shows the tracked centroids
(red, turning blue once the detection is stable), the reprojected quadrotor
model (green circles) and the estimated body X axis (red arrow). The pose estimate itself is on `/epa/pose`.

<!-- ## Citation -->
