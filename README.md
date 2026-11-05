# ORBVI ROS Bridge

[中文版本](README.zh-CN.md)

Installing the bundled 4VL board runtime? See the
[deb installation and bridge quick start](third_party/orbvi_sdk/deb/README.md)
for package verification, board installation, startup checks and rollback.

ORBVI ROS Bridge publishes ORBVI device data into ROS1 or ROS2 through the
ORBVI Host SDK. It is intended for application developers who need camera, IMU,
MID360 LiDAR, disparity, Host-derived depth, board-generated panorama/BEV and
VIO topics on Linux hosts.

The repository includes:

- ROS1 Noetic and ROS2 Foxy/Jazzy bridge nodes
- 4V and 4VL bridge launch files, plus RViz and rqt viewer launch files
- Minimal `livox_ros_driver2` CustomMsg definitions required by the bridge
- MID360 and full-topic rate check tooling

## Supported Platforms

| Platform | OS | ROS | Status |
| --- | --- | --- | --- |
| x86_64 / amd64 workstation | Ubuntu 20.04 | ROS1 Noetic | Supported |
| x86_64 / amd64 workstation | Ubuntu 20.04 | ROS2 Foxy | Supported |
| x86_64 / amd64 workstation | Ubuntu 24.04 | ROS2 Jazzy | Tested |
| aarch64 / arm64 ORBVI host | Ubuntu 20.04 | ROS1 Noetic | Supported |
| aarch64 / arm64 ORBVI host | Ubuntu 20.04 | ROS2 Foxy | Supported |

## Topic Contract

ROS1 and ROS2 use the same topic names. ROS2 message packages use the matching
`::msg` namespaces.

For rectified topics, the direction path component is the SDK Core
`pair_direction`: both `left` and `right` images in one `pair_id` therefore use
the same direction. `source_camera_direction` remains physical-source metadata
for calibration and diagnostics and does not select the topic.

| Topic | ROS1 type | Notes |
| --- | --- | --- |
| `/orbvi/raw/camera_<id>/image` | `sensor_msgs/Image` | Decoded BGR image when enabled |
| `/orbvi/raw/camera_<id>/image/compressed` | `sensor_msgs/CompressedImage` | Raw fisheye compressed image transport |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image` | `sensor_msgs/Image` | Decoded rectified image when enabled |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image/compressed` | `sensor_msgs/CompressedImage` | Rectified stereo pair compressed image transport |
| `/orbvi/pano/image` | `sensor_msgs/Image` | Board `pano_display_stream`, with Host derivation only when the board explicitly reports the stream unsupported |
| `/orbvi/ipm/image` | `sensor_msgs/Image` | Board `bev_display_stream` ground-plane BEV (`1000x1000`, `bgr8`, target 20 Hz), with Host derivation only when the board explicitly reports the stream unsupported |
| `/orbvi/imu` | `sensor_msgs/Imu` | Device IMU |
| `/orbvi/lidar/custom` | `livox_ros_driver2/CustomMsg` | MID360 point cloud stream |
| `/orbvi/lidar/imu` | `sensor_msgs/Imu` | MID360 IMU; acceleration m/s², angular velocity rad/s |
| `/orbvi/disparity` | `sensor_msgs/Image` | Disparity image |
| `/orbvi/depth` | `sensor_msgs/Image` | Generated depth image |
| `/orbvi/<front\|right\|rear\|left>/depth/viz` | `sensor_msgs/Image` | Directional depth visualization; optionally includes a 5x5 metric grid |
| `/orbvi/depth/panorama` | `sensor_msgs/Image` | `32FC1` spherical range generated after transforming all four stereo depth tiles by their full rig poses (rotation and translation) |
| `/orbvi/depth/panorama/viz` | `sensor_msgs/Image` | Depth-panorama visualization |
| `/orbvi/depth/points` | `sensor_msgs/PointCloud2` | Depth-derived point cloud; the default `ros` output frame applies each stereo pair's full rig pose |
| `/orbvi/cloud_registered` | `sensor_msgs/PointCloud2` | Omni-LIVO registered world-frame XYZI + RGB cloud; optional 4VL stream |
| `/orbvi/localization/odometry` | `nav_msgs/Odometry` | Optimized fused pose (VINS or Omni-LIVO) |
| `/orbvi/localization/path` | `nav_msgs/Path` | Bounded trajectory accumulated from optimized fused poses |
| `/orbvi/localization/imu_prediction` | `nav_msgs/Odometry` | IMU-propagated pose (VINS or Omni-LIVO) |

The rectified direction names the stereo pair's viewing direction, not the
physical source fisheye. Images are published in the fixed order
`front/left`, `front/right`, `right/left`, `right/right`, `rear/left`,
`rear/right`, `left/left`, `left/right` with these pixel sources:

| Stereo direction | Left-eye source | Right-eye source |
| --- | --- | --- |
| `front` | Right half of front fisheye | Left half of right fisheye |
| `right` | Right half of right fisheye | Left half of rear fisheye |
| `rear` | Right half of rear fisheye | Left half of left fisheye |
| `left` | Right half of left fisheye | Left half of front fisheye |

## Install Dependencies

Install ROS and build tools on the bridge host.

ROS1 Noetic:

```bash
sudo apt update
sudo apt install -y \
  cmake libboost-dev libyaml-cpp-dev libopencv-dev \
  ros-noetic-roscpp \
  ros-noetic-std-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-message-generation
```

ROS2 Foxy or Jazzy:

```bash
export ROS_DISTRO=foxy  # or jazzy
sudo apt update
sudo apt install -y \
  cmake libboost-dev libyaml-cpp-dev libopencv-dev \
  python3-colcon-common-extensions \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-nav-msgs \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-rosidl-default-generators
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

## Build ROS2

```bash
mkdir -p ~/orbvi_ros2_ws/src
cd ~/orbvi_ros2_ws/src
git clone <this-repo-url> orbvi_ros_bridge
cd ..

source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Start The Bridge

Pick the launch file that matches the device model. Both enable every
capability of that model by default. To trim the output, override `streams`.
Common arguments are listed in [Launch Arguments](docs/LAUNCH_ARGUMENTS.md).

The device address defaults to `192.168.3.100`. If your device uses another
address, append `host:=<device-ip>` to the commands below.

| Device | ROS1 launch | ROS2 launch | Default `streams` |
| --- | --- | --- | --- |
| 4V (four cameras, no LiDAR) | `orbvi_ros_bridge_4v.launch` | `orbvi_ros_bridge_4v.launch.py` | `raw,rectified,pano,bev,imu,disparity,depth,localization` |
| 4VL (four cameras with LiDAR) | `orbvi_ros_bridge_4vl.launch` | `orbvi_ros_bridge_4vl.launch.py` | `raw,rectified,pano,bev,imu,lidar,lidar_imu,disparity,depth,localization,registered_cloud` |

4V:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4v.launch.py
```

4VL:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch.py
```

First topics to check: `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`,
`/orbvi/localization/odometry`, `/orbvi/pano/image` and `/orbvi/ipm/image`. 4VL also
publishes `/orbvi/lidar/custom` and `/orbvi/lidar/imu`.

Current firmware generates panorama and BEV on the board; the host only decodes
and publishes them. Host-side derivation is a fallback for older firmware that
reports a stream as unsupported. If you do not need them, remove `pano,bev` from
`streams`, for example:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch \
  streams:=raw,rectified,imu,disparity,depth,localization
```

## Documentation

| Topic | Document |
| --- | --- |
| Launch arguments, depth grid overlay, VIO path, pose/TF summary, ROS2 QoS | [Launch Arguments](docs/LAUNCH_ARGUMENTS.md) |
| BEV/IPM and panorama behavior, Host fallback, depth assistance | [Panorama and BEV](docs/PANORAMA_BEV.md) |
| RViz, rqt (topic rates and images), ROS2 shared-memory transport | [View Data](docs/VIEWING.md) |
| MID360 data check, full topic rate check, FAQ | [Checks and Troubleshooting](docs/TROUBLESHOOTING.md) |
| Frame definitions and extrinsic calibration | [TF and extrinsic calibration](docs/TF.md) |
| Robot mounting, robot TF and ground IPM | [Robot TF and IPM](docs/ROBOT_INTEGRATION.md) |

## License

MIT
