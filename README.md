# ORBVI ROS Bridge

ORBVI ROS Bridge is the ROS1 Noetic package for ORBVI devices. It connects to
the device through the ORBVI Host SDK and publishes camera, IMU, MID360 LiDAR,
depth and VIO data as standard ROS topics.

This repository is the public bridge layer only. It contains ROS nodes, launch
files, message conversion code and topic-rate checking tools. The ORBVI Host
SDK, board runtime, OTA service, factory packages and internal deployment tools
are intentionally kept outside this repository.

## Supported Platforms

| Platform | OS | ROS | Status |
| --- | --- | --- | --- |
| x86_64 / amd64 workstation | Ubuntu 20.04 | ROS1 Noetic | Supported |
| aarch64 / arm64 edge host | Ubuntu 20.04 | ROS1 Noetic | Supported |

The bridge source is the same on both platforms. Install or build an ORBVI Host
SDK package for the matching CPU architecture, then pass its install prefix to
catkin with `-DORBVI_HOST_SDK_ROOT=...`.

## Published Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/orbvi/raw/camera_<id>/compressed` | `sensor_msgs/CompressedImage` | Raw fisheye MJPEG, split per camera when bundle metadata is available |
| `/orbvi/raw/camera_<id>/image` | `sensor_msgs/Image` | Decoded BGR image, only when decoded image mode is enabled |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/compressed` | `sensor_msgs/CompressedImage` | Rectified stereo pair images |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image` | `sensor_msgs/Image` | Decoded rectified image, only when decoded image mode is enabled |
| `/orbvi/imu` | `sensor_msgs/Imu` | Device IMU |
| `/orbvi/lidar/custom` | `livox_ros_driver2/CustomMsg` | MID360 point cloud stream |
| `/orbvi/lidar/imu` | `sensor_msgs/Imu` | MID360 IMU stream |
| `/orbvi/disparity` | `sensor_msgs/Image` | Disparity image |
| `/orbvi/depth` | `sensor_msgs/Image` | Generated depth image when depth output is enabled |
| `/orbvi/depth/viz` | `sensor_msgs/Image` | Depth visualization image |
| `/orbvi/depth/points` | `sensor_msgs/PointCloud2` | Depth-derived point cloud |
| `/orbvi/vio/odometry` | `nav_msgs/Odometry` | Optimized VIO pose |
| `/orbvi/vio/imu_prediction` | `nav_msgs/Odometry` | IMU-propagated VIO pose |

## Dependencies

Install ROS1 Noetic and common ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-roscpp \
  ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs \
  libopencv-dev
```

`livox_ros_driver2` message definitions must be available in the same catkin
workspace. The ORBVI Host SDK headers and library must also be installed for
the current platform.

## Build

Create a catkin workspace:

```bash
mkdir -p ~/orbvi_ros_ws/src
cd ~/orbvi_ros_ws/src
git clone <this-repo-url> orbvi_ros_bridge
cd ..
source /opt/ros/noetic/setup.bash
```

Build with an installed Host SDK:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DORBVI_HOST_SDK_ROOT=/opt/orbvi-sdk
```

If the Host SDK include and library paths are separate:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DORBVI_HOST_SDK_INCLUDE_DIR=/path/to/include \
  -DORBVI_HOST_SDK_LIBRARY=/path/to/liborbvi_sdk.so
```

For internal development against a local Host SDK source tree:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DORBVI_HOST_SDK_SOURCE_DIR=/path/to/orbvi-host-sdk/sdk/cpp
```

Source the workspace after a successful build:

```bash
source ~/orbvi_ros_ws/devel/setup.bash
```

## Quick Start

Start all streams:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> \
  control_port:=18088
```

Start only MID360 topics:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch \
  host:=<device-ip>
```

Start only camera, IMU, disparity, depth and VIO topics:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_visual.launch \
  host:=<device-ip>
```

For low-latency monitoring, keep image data compressed and use shallow queues:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> \
  image_mode:=raw-only \
  queue_size:=2 \
  max_receive_queue_depth:=4 \
  max_decode_queue_depth:=4
```

## Launch Files

### `orbvi_ros_bridge.launch`

General-purpose bridge entry. It can publish all ORBVI streams and exposes the
full set of tuning arguments.

Common arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `host` | `127.0.0.1` | ORBVI device IP or hostname |
| `control_port` | `18088` | ORBVI SDK control port |
| `streams` | `raw,rectified,imu,lidar,lidar_imu,disparity,depth,vio` | Comma-separated stream list |
| `topic_prefix` | `/orbvi` | ROS topic prefix |
| `image_mode` | `raw-and-decoded` | `raw-only`, `decoded` or `raw-and-decoded` |
| `queue_size` | `4` | ROS publisher queue size |
| `max_receive_queue_depth` | `8` | Host SDK receive queue depth |
| `max_decode_queue_depth` | `8` | Host SDK decode queue depth |
| `publish_depth` | `auto` | `auto`, `true` or `false` |
| `publish_depth_viz` | `true` | Publish `/orbvi/depth/viz` |
| `publish_depth_pointcloud` | `true` | Publish `/orbvi/depth/points` |

### `orbvi_ros_bridge_mid360.launch`

MID360-only entry. It subscribes to `lidar,lidar_imu` and publishes:

- `/orbvi/lidar/custom`
- `/orbvi/lidar/imu`

### `orbvi_ros_bridge_visual.launch`

Visual/VIO entry. It subscribes to `raw,rectified,imu,disparity,depth,vio` and
does not request MID360 streams. Use this when testing camera, IMU, depth and
VIO without LiDAR load.

## MID360 Data Check

After starting `orbvi_ros_bridge_mid360.launch`, check the topic list:

```bash
rostopic list | grep '^/orbvi/lidar'
```

Check rates manually:

```bash
rostopic hz /orbvi/lidar/custom --window 30
rostopic hz /orbvi/lidar/imu --window 100
```

Typical healthy values are around 10 Hz for `/orbvi/lidar/custom` and around
200 Hz for `/orbvi/lidar/imu`. Exact rates depend on device configuration and
host load.

Run the bundled check script on the ROS bridge host:

```bash
scripts/orbvi_ros_bridge_rate_check.sh \
  --bridge-setup ~/orbvi_ros_ws/devel/setup.bash \
  --topics /orbvi/lidar/custom,/orbvi/lidar/imu \
  --sample-sec 20
```

Start a temporary MID360 bridge on a remote ROS host and measure there:

```bash
SSH_PASSWORD=<password> scripts/orbvi_ros_bridge_rate_check.sh \
  --ssh <user>@<bridge-host> \
  --bridge-setup ~/orbvi_ros_ws/devel/setup.bash \
  --device-host <device-ip> \
  --start-bridge \
  --streams lidar,lidar_imu \
  --topics /orbvi/lidar/custom,/orbvi/lidar/imu \
  --sample-sec 20
```

The script measures ROS topics from the ROS bridge host's local ROS master. The
device host is only used when the script starts a temporary bridge process.

## Full Topic Rate Check

Check all `/orbvi/*` topics from the ROS bridge host:

```bash
scripts/orbvi_ros_bridge_rate_check.sh \
  --bridge-setup ~/orbvi_ros_ws/devel/setup.bash \
  --sample-sec 20
```

The script writes:

- `summary.log`: human-readable run log
- `rates.csv`: topic, type, status and measured rate
- `topics/*.hz.log`: raw `rostopic hz` output per topic

## View Data

Image topics can be viewed with `rqt_image_view`:

```bash
rosrun rqt_image_view rqt_image_view /orbvi/raw/camera_0/compressed
```

Point clouds and odometry can be viewed with RViz:

```bash
rviz
```

Useful RViz topics:

- `/orbvi/lidar/custom`
- `/orbvi/depth/points`
- `/orbvi/vio/odometry`

## Troubleshooting

### No topics are published

Check that ROS and the workspace are sourced in the same shell:

```bash
source /opt/ros/noetic/setup.bash
source ~/orbvi_ros_ws/devel/setup.bash
rosnode list
```

Check that the device control port is reachable from the bridge host:

```bash
nc -vz <device-ip> 18088
```

### `livox_ros_driver2/CustomMsg` cannot be loaded

Build `livox_ros_driver2` message definitions in the same catkin workspace,
then source `devel/setup.bash` again.

### MID360 rate is missing

Use the MID360 launch first:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
```

Then check:

```bash
rostopic type /orbvi/lidar/custom
rostopic hz /orbvi/lidar/custom --window 30
```

If the topic type exists but rate is empty, keep the bridge running and inspect
device-side sensor state with the device management tooling.

### Image decoding uses too much CPU

Use compressed-only output:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> \
  image_mode:=raw-only
```

### Depth topics are missing

Depth output depends on disparity frames and valid calibration from the Host
SDK. Start with:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_visual.launch \
  host:=<device-ip> \
  publish_depth:=true
```

Then check `/orbvi/disparity`, `/orbvi/depth` and the bridge log.

## Open Source Boundary

Keep the following out of this repository before public release:

- Device rootfs, factory package and OTA implementation
- Internal board service files and deployment scripts
- Private IPs, credentials, CI variables, webhook URLs and customer data
- Proprietary calibration files, models and runtime logs
- Host SDK implementation if it is not part of the same public release

## License

MIT
