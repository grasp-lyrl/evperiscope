# Setting up from scratch

Step-by-step version of the summary in the [top-level README](../README.md).
Written against Ubuntu 22.04 with ROS 2 Humble.

## 1. GCC 13

Required by `dv-processing`.
Ubuntu 22.04 ships GCC 11, so install 13 from the toolchain PPA:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update && sudo apt install -y gcc-13 g++-13
```

## 2. System libraries

```bash
sudo apt install -y libopencv-dev libeigen3-dev libhdf5-dev
```

## 3. dv-processing 2.0.3

The event camera library, built from source with the compiler from step 1.
`boost-inivation` and `libcaer-dev` come from iniVation's PPA:

```bash
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt update && sudo apt install -y \
    boost-inivation libboost-all-dev libcaer-dev \
    libfmt-dev liblz4-dev libzstd-dev libssl-dev libusb-1.0-0-dev

git clone -b 2.0.3 https://gitlab.com/inivation/dv/dv-processing.git
cd dv-processing && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr \
      -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13 ..
make -j$(nproc) && sudo make install
```

`-DCMAKE_INSTALL_PREFIX=/usr` matters: it puts the CMake config where
`find_package(dv-processing)` looks by default.


## 4. Workspace

```bash
mkdir -p ~/epa_ws/src && cd ~/epa_ws/src

git clone https://github.com/grasp-lyrl/evperiscope.git
git clone https://github.com/grasp-lyrl/dv-ros2-wrapper.git

cd ~/epa_ws
rosdep install --from-paths src --ignore-src -r -y
```

`dv-ros2-wrapper` provides the event camera driver (`dv_ros2_capture`) and the
`dv_ros2_msgs/EventArray` message type this package subscribes to. It also
carries `scripts/dv_ros2_bag_to_h5.py`, used to convert bags to HDF5.

## 5. Build

The whole workspace has to use GCC 13, since `dv-processing` is linked by this
package:

```bash
cd ~/epa_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13
source install/setup.bash
```

To avoid repeating the flags, put them in a `colcon_defaults.yaml` at the
workspace root instead:

```yaml
build:
  cmake-args:
    - -DCMAKE_C_COMPILER=gcc-13
    - -DCMAKE_CXX_COMPILER=g++-13
```
