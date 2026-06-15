# ORBVI ROS1 Bridge

`orbvi_ros_bridge` is a ROS1 Noetic wrapper around the ORBVI Host SDK. It connects
to an ORBVI device through the SDK control/stream transport and republishes data
as standard ROS topics.

This repository is intentionally thin:

- It contains ROS1 node, launch, frame conversion and rate-check tooling.
- It depends on the ORBVI Host SDK as an external C++ library.
- It does not contain board runtime services, OTA agents, device drivers,
  factory packaging, internal CI jobs or private deployment scripts.

## Published Topics

- `/orbvi/raw/camera_<id>/compressed`: `sensor_msgs/CompressedImage`
- `/orbvi/raw/camera_<id>/image`: `sensor_msgs/Image`, when decoded image mode is enabled
- `/orbvi/rectified/<front|right|rear|left>/<left|right>/compressed`: `sensor_msgs/CompressedImage`
- `/orbvi/rectified/<front|right|rear|left>/<left|right>/image`: `sensor_msgs/Image`, when decoded image mode is enabled
- `/orbvi/imu`: `sensor_msgs/Imu`
- `/orbvi/lidar/custom`: `livox_ros_driver2/CustomMsg`
- `/orbvi/lidar/imu`: `sensor_msgs/Imu`
- `/orbvi/disparity`: `sensor_msgs/Image`
- `/orbvi/depth`: `sensor_msgs/Image`
- `/orbvi/depth/viz`: `sensor_msgs/Image`
- `/orbvi/depth/points`: `sensor_msgs/PointCloud2`
- `/orbvi/vio/odometry`: `nav_msgs/Odometry`
- `/orbvi/vio/imu_prediction`: `nav_msgs/Odometry`

## Dependencies

- Ubuntu 20.04 with ROS1 Noetic
- `ros-noetic-roscpp`, `ros-noetic-sensor-msgs`, `ros-noetic-nav-msgs`
- `livox_ros_driver2` messages available in the same catkin workspace
- OpenCV development package
- ORBVI Host SDK headers and library

## Build With Installed Host SDK

Assume the Host SDK is installed under `/opt/orbvi-sdk`:

```bash
mkdir -p ~/orbvi_ros1_ws/src
cd ~/orbvi_ros1_ws/src
git clone <this-repo-url> orbvi_ros1_bridge
cd ..
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DORBVI_HOST_SDK_ROOT=/opt/orbvi-sdk
```

If the include and library paths are separate:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DORBVI_HOST_SDK_INCLUDE_DIR=/path/to/include \
  -DORBVI_HOST_SDK_LIBRARY=/path/to/liborbvi_sdk.so
```

## Development Build Against Host SDK Source

For internal development, the bridge can point to a local Host SDK CMake project:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DORBVI_HOST_SDK_SOURCE_DIR=/path/to/orbvi-host-sdk/sdk/cpp
```

This mode is for development only. Public releases should consume packaged Host
SDK headers and libraries.

## Run

```bash
source /opt/ros/noetic/setup.bash
source ~/orbvi_ros1_ws/devel/setup.bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch host:=<device-ip> control_port:=18088
```

Useful launch arguments:

- `streams`: default `raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio`
- `topic_prefix`: default `/orbvi`
- `image_mode`: `raw-only`, `decoded` or `raw-and-decoded`
- `require_streaming_transport`: default `true`
- `allow_sample_endpoint_fallback`: default `false`
- `queue_size`: ROS publisher queue size
- `max_receive_queue_depth`: Host SDK receive queue depth
- `max_decode_queue_depth`: Host SDK decode queue depth
- `decode_threads`: optional `ORBVI_SDK_JPEG_DECODE_THREADS`

For low-latency monitoring, start with:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> \
  control_port:=18088 \
  image_mode:=raw-only \
  queue_size:=2 \
  max_receive_queue_depth:=4 \
  max_decode_queue_depth:=4
```

## Rate Check

Run from the ROS bridge host:

```bash
scripts/orbvi_ros_bridge_rate_check.sh \
  --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
  --sample-sec 20
```

Start a temporary bridge and measure all `/orbvi/*` topic rates from another
machine:

```bash
SSH_PASSWORD=<password> scripts/orbvi_ros_bridge_rate_check.sh \
  --ssh <user>@<bridge-host> \
  --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
  --device-host <device-ip> \
  --start-bridge \
  --sample-sec 20
```

Check only MID360 bridge topics:

```bash
SSH_PASSWORD=<password> scripts/orbvi_ros_bridge_rate_check.sh \
  --ssh <user>@<bridge-host> \
  --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
  --topics /orbvi/lidar/custom,/orbvi/lidar/imu
```

The script writes a CSV and per-topic `rostopic hz` logs under its output
directory. All rates are measured from the ROS bridge host's local ROS master.
The device IP is only needed if the script starts a temporary bridge.

## Open Source Boundary

Before publishing this repository, keep the following out of tree:

- Device rootfs, factory package and OTA implementation
- Internal board service files and deployment scripts
- Private IPs, credentials, CI variables, webhook URLs and customer data
- Proprietary calibration files, models and runtime logs
- Host SDK implementation if it is not part of the same public release

## License

MIT
