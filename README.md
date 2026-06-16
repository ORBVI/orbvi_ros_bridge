# ORBVI ROS Bridge

ORBVI ROS Bridge publishes ORBVI device data into ROS1 or ROS2 through the
ORBVI Host SDK. It is the open bridge package for application developers who
need camera, IMU, MID360 LiDAR, disparity, depth and VIO topics on Linux hosts.

The repository includes:

- ROS1 Noetic and ROS2 Foxy bridge nodes
- Launch files for full-data, visual, and visual + MID360 workflows
- Minimal `livox_ros_driver2` CustomMsg definitions required by the bridge
- Public ORBVI Host SDK headers and Linux x86_64/aarch64 libraries
- MID360 and full-topic rate check tooling
- Multi-architecture CI coverage for x86_64 and arm64 Linux hosts

Board runtime, OTA implementation, factory packages, private deployment scripts,
credentials and customer data must stay outside this public repository.

## Supported Platforms

| Platform | OS | ROS | Status |
| --- | --- | --- | --- |
| x86_64 / amd64 workstation | Ubuntu 20.04 | ROS1 Noetic | Supported |
| x86_64 / amd64 workstation | Ubuntu 20.04 | ROS2 Foxy | Supported |
| aarch64 / arm64 ORBVI host | Ubuntu 20.04 | ROS1 Noetic | Supported |
| aarch64 / arm64 ORBVI host | Ubuntu 20.04 | ROS2 Foxy | Supported |

The bundled Host SDK libraries are built with GCC 9. Use GCC 9 when rebuilding
or replacing the Host SDK binary package. The ROS bridge package itself does
not require a fixed compiler version and can use the compiler provided by the
target ROS build environment.

- `third_party/orbvi_sdk/libs/linux-x86_64`
- `third_party/orbvi_sdk/libs/linux-aarch64`

## SDK Bundle Model

The bridge follows a product SDK layout: users clone one repository, build the
included ROS packages and get a working bridge without rebuilding the Host SDK.

- `third_party/orbvi_sdk/include/orbvi_sdk` contains the public Host SDK C++
  headers used by both ROS1 and ROS2.
- `third_party/orbvi_sdk/libs/linux-x86_64` contains the Ubuntu x86_64 SDK
  libraries.
- `third_party/orbvi_sdk/libs/linux-aarch64` contains the Ubuntu arm64 SDK
  libraries for ORBVI hosts.
- `livox_ros_driver2` contains the minimal MID360 `CustomMsg` message
  definitions used by the bridge. It is intentionally a message-only package.
- CMake selects the matching library directory from `CMAKE_SYSTEM_PROCESSOR`.
- `libturbojpeg` is linked into `liborbvi_sdk.so`, so bridge users do not need
  to install TurboJPEG just to run the ROS bridge.

Advanced users can still override the SDK with `ORBVI_HOST_SDK_ROOT`,
`ORBVI_HOST_SDK_INCLUDE_DIR`, `ORBVI_HOST_SDK_LIBRARY` or
`ORBVI_HOST_SDK_SOURCE_DIR`.

## Topic Contract

ROS1 and ROS2 use the same topic names. ROS2 message packages use the matching
`::msg` namespaces.

| Topic | ROS1 type | Notes |
| --- | --- | --- |
| `/orbvi/raw/camera_<id>/compressed` | `sensor_msgs/CompressedImage` | Raw fisheye MJPEG |
| `/orbvi/raw/camera_<id>/image` | `sensor_msgs/Image` | Decoded BGR image when enabled |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/compressed` | `sensor_msgs/CompressedImage` | Rectified stereo pair images |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image` | `sensor_msgs/Image` | Decoded rectified image when enabled |
| `/orbvi/imu` | `sensor_msgs/Imu` | Device IMU |
| `/orbvi/lidar/custom` | `livox_ros_driver2/CustomMsg` | MID360 point cloud stream |
| `/orbvi/lidar/imu` | `sensor_msgs/Imu` | MID360 IMU stream |
| `/orbvi/disparity` | `sensor_msgs/Image` | Disparity image |
| `/orbvi/depth` | `sensor_msgs/Image` | Generated depth image |
| `/orbvi/depth/viz` | `sensor_msgs/Image` | Depth visualization image |
| `/orbvi/depth/points` | `sensor_msgs/PointCloud2` | Depth-derived point cloud |
| `/orbvi/vio/odometry` | `nav_msgs/Odometry` | Optimized VIO pose |
| `/orbvi/vio/imu_prediction` | `nav_msgs/Odometry` | IMU-propagated VIO pose |

## Install Dependencies

Install ROS and build tools on the bridge host.

ROS1 Noetic:

```bash
sudo apt update
sudo apt install -y \
  cmake libboost-dev libopencv-dev \
  ros-noetic-roscpp \
  ros-noetic-std-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-message-generation
```

ROS2 Foxy:

```bash
sudo apt update
sudo apt install -y \
  cmake libboost-dev libopencv-dev \
  python3-colcon-common-extensions \
  ros-foxy-rclcpp \
  ros-foxy-std-msgs \
  ros-foxy-sensor-msgs \
  ros-foxy-nav-msgs \
  ros-foxy-rosidl-default-generators
```

The repository already includes the `livox_ros_driver2` message definitions
needed by `orbvi_ros_bridge`, so a fresh clone in a ROS workspace can compile
directly.

## Build ROS1

```bash
mkdir -p ~/orbvi_ros1_ws/src
cd ~/orbvi_ros1_ws/src
git clone <this-repo-url> orbvi_ros_bridge
cd ..

source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Use a custom SDK instead of the bundled SDK when needed:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DORBVI_HOST_SDK_ROOT=/opt/orbvi-sdk
```

For SDK source development:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DORBVI_HOST_SDK_SOURCE_DIR=/path/to/orbvi-host-sdk/sdk/cpp
```

## Build ROS2

```bash
mkdir -p ~/orbvi_ros2_ws/src
cd ~/orbvi_ros2_ws/src
git clone <this-repo-url> orbvi_ros_bridge
cd ..

source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Use a custom SDK with the same CMake options:

```bash
colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DORBVI_HOST_SDK_ROOT=/opt/orbvi-sdk
```

## Start The Bridge

ROS1 all streams:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip>
```

ROS2 all streams:

```bash
ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge.launch.py \
  host:=<device-ip>
```

## Scenario Quickstart

Use the launch file that matches the required runtime capability. Public topics
are fixed under `/orbvi`.

| Scenario | ROS1 command | ROS2 command | Expected first check |
| --- | --- | --- | --- |
| Basic all-data bridge | `roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge.launch.py host:=<device-ip>` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry`, `/orbvi/lidar/custom` |
| Camera, depth and VIO | `roslaunch orbvi_ros_bridge orbvi_ros_bridge_visual.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge_visual.launch.py host:=<device-ip>` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry` |
| Camera, depth, VIO and MID360 | `roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge_mid360.launch.py host:=<device-ip>` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry`, `/orbvi/lidar/custom`, `/orbvi/lidar/imu` |

Use `orbvi_ros_bridge_visual.launch` when the host only needs visual raw data,
depth and VIO. Use `orbvi_ros_bridge_mid360.launch` when the same visual
capability should run together with MID360 point cloud and MID360 IMU.

## Visual And MID360 Data Check

Start the visual + MID360 entry.

ROS1:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
```

ROS2:

```bash
ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge_mid360.launch.py host:=<device-ip>
```

Check topic presence and type from the bridge host:

```bash
rostopic list | grep '^/orbvi/lidar'
rostopic type /orbvi/lidar/custom
rostopic type /orbvi/lidar/imu
```

Measure ROS1 rates:

```bash
rostopic hz /orbvi/lidar/custom --window 30
rostopic hz /orbvi/lidar/imu --window 100
```

Typical healthy values are around 10 Hz for `/orbvi/lidar/custom` and around
200 Hz for `/orbvi/lidar/imu`. Exact values depend on MID360 configuration,
network quality and host load.

## Full Topic Rate Check

The script checks topics from the ROS bridge host's local ROS master. It should
run on the same machine that sources the ROS bridge workspace.

MID360 topic subset:

```bash
scripts/orbvi_ros_bridge_rate_check.sh \
  --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
  --topics /orbvi/lidar/custom,/orbvi/lidar/imu \
  --sample-sec 20
```

All `/orbvi/*` topics:

```bash
scripts/orbvi_ros_bridge_rate_check.sh \
  --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
  --sample-sec 20
```

Start a temporary bridge and measure locally on a remote ROS1 host:

```bash
SSH_PASSWORD=<password> scripts/orbvi_ros_bridge_rate_check.sh \
  --ssh <user>@<bridge-host> \
  --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
  --device-host <device-ip> \
  --start-bridge \
  --topics /orbvi/lidar/custom,/orbvi/lidar/imu \
  --sample-sec 20
```

Outputs are written to a timestamped directory:

- `summary.log`
- `rates.csv`
- `topics/*.hz.log`

## Launch Files

Launch files are grouped by ROS version and runtime scenario.

| Launch file | ROS | Purpose |
| --- | --- | --- |
| `orbvi_ros_bridge.launch` | ROS1 | Default full raw-data entry: visual data, depth, VIO and MID360 |
| `orbvi_ros_bridge_visual.launch` | ROS1 | Visual entry: four raw decoded images, rectified images, device IMU, disparity, depth and VIO |
| `orbvi_ros_bridge_mid360.launch` | ROS1 | Visual entry plus MID360 point cloud and MID360 IMU |
| `ros2/orbvi_ros_bridge.launch.py` | ROS2 | Default full raw-data entry: visual data, depth, VIO and MID360 |
| `ros2/orbvi_ros_bridge_visual.launch.py` | ROS2 | Visual entry: four raw decoded images, rectified images, device IMU, disparity, depth and VIO |
| `ros2/orbvi_ros_bridge_mid360.launch.py` | ROS2 | Visual entry plus MID360 point cloud and MID360 IMU |

Common parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `host` | `127.0.0.1` | ORBVI device IP or hostname |
| `control_port` | `18088` | ORBVI SDK control port |

Topic names are fixed under `/orbvi`.

## View Data

ROS1 images:

```bash
rosrun rqt_image_view rqt_image_view /orbvi/raw/camera_0/compressed
```

ROS2 images:

```bash
ros2 run rqt_image_view rqt_image_view /orbvi/raw/camera_0/compressed
```

Point clouds and odometry can be viewed in RViz:

- `/orbvi/lidar/custom`
- `/orbvi/depth/points`
- `/orbvi/vio/odometry`

## Continuous Integration

Maintainer CI validates the same clone-and-build layout documented above for
ROS1 Noetic and ROS2 Foxy on both x86_64 and arm64 Linux hosts. Normal
pipelines only compile and test the current ROS packages with the prebuilt CI
images selected by `CI_IMAGE_TAG_SUFFIX`. Docker image refresh jobs are manual
maintenance jobs and should only be run when the CI Dockerfile or base
dependency set changes.

ROS bridge jobs use the default compiler from each ROS build environment; Host
SDK binaries should continue to be rebuilt with GCC 9 before being refreshed in
`third_party/orbvi_sdk`.

## Refresh Bundled Host SDK

Build the Host SDK release libraries in the main workspace first. The release
SDK should be built on Ubuntu 20.04 with GCC 9 and PIC static TurboJPEG linked
into `liborbvi_sdk.so`.

After producing x86_64 and aarch64 install trees, refresh this repository:

```bash
rsync -a /path/to/sdk-x86_64/include/orbvi_sdk/ \
  third_party/orbvi_sdk/include/orbvi_sdk/
rsync -a /path/to/sdk-x86_64/lib/ \
  third_party/orbvi_sdk/libs/linux-x86_64/
rsync -a /path/to/sdk-aarch64/lib/ \
  third_party/orbvi_sdk/libs/linux-aarch64/
```

Verify the refreshed binaries:

```bash
file third_party/orbvi_sdk/libs/linux-x86_64/liborbvi_sdk.so.0.1.0
file third_party/orbvi_sdk/libs/linux-aarch64/liborbvi_sdk.so.0.1.0
readelf -d third_party/orbvi_sdk/libs/linux-x86_64/liborbvi_sdk.so.0.1.0 | grep NEEDED
```

`readelf` should not show `libturbojpeg` in the runtime dependencies.

## FAQ

### No topics are published

Check that ROS and the bridge workspace are sourced in the same shell:

```bash
source /opt/ros/noetic/setup.bash
source ~/orbvi_ros1_ws/devel/setup.bash
rosnode list
```

Check the device control port from the bridge host:

```bash
nc -vz <device-ip> 18088
```

### `livox_ros_driver2/CustomMsg` cannot be loaded

Make sure the whole repository was cloned into the workspace `src` directory
and rebuild the workspace. This repository includes a message-only
`livox_ros_driver2` package for `CustomMsg`; it does not include Livox driver
nodes or device configuration tools.

### MID360 topic exists but rate is empty

Run the MID360 launch first, then measure from the same bridge host:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
rostopic hz /orbvi/lidar/custom --window 30
```

If the type exists but no samples arrive, keep the bridge running and inspect
device-side MID360 state with device management tooling.

### Depth topics are missing

Depth output depends on disparity frames and valid calibration from the Host
SDK:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_visual.launch \
  host:=<device-ip>
```

Then check `/orbvi/disparity`, `/orbvi/depth` and the bridge log.

### The wrong SDK architecture is linked

Check the selected library:

```bash
ldd devel/lib/orbvi_ros_bridge/orbvi_ros_bridge_node | grep orbvi_sdk
```

Use `-DORBVI_HOST_SDK_LIBRARY=/path/to/liborbvi_sdk.so` to force a specific SDK
library when building custom layouts.

## Open Source Boundary

Keep these out of the public repository:

- Device rootfs, factory package and OTA implementation
- Internal board service files and deployment scripts
- Private IPs, credentials, CI variables, webhook URLs and customer data
- Proprietary calibration files, models and runtime logs
- Host SDK implementation if it is not part of the same public release

## License

MIT
