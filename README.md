# ORBVI ROS Bridge

[中文版本](README.zh-CN.md)

ORBVI ROS Bridge publishes ORBVI device data into ROS1 or ROS2 through the
ORBVI Host SDK. It is intended for application developers who need camera, IMU,
MID360 LiDAR, disparity, Host-derived depth, Host-derived panorama and VIO
topics on Linux hosts.

The repository includes:

- ROS1 Noetic and ROS2 Foxy/Jazzy bridge nodes
- Launch files for visual, MID360, and panorama workflows
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

| Topic | ROS1 type | Notes |
| --- | --- | --- |
| `/orbvi/raw/camera_<id>/image` | `sensor_msgs/Image` | Decoded BGR image when enabled |
| `/orbvi/raw/camera_<id>/image/compressed` | `sensor_msgs/CompressedImage` | Raw fisheye compressed image transport |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image` | `sensor_msgs/Image` | Decoded rectified image when enabled |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image/compressed` | `sensor_msgs/CompressedImage` | Rectified stereo pair compressed image transport |
| `/orbvi/pano/image` | `sensor_msgs/Image` | Host SDK panorama stitched from `raw_fisheye_stream` when `pano` is enabled |
| `/orbvi/imu` | `sensor_msgs/Imu` | Device IMU |
| `/orbvi/lidar/custom` | `livox_ros_driver2/CustomMsg` | MID360 point cloud stream |
| `/orbvi/lidar/imu` | `sensor_msgs/Imu` | MID360 IMU stream |
| `/orbvi/disparity` | `sensor_msgs/Image` | Disparity image |
| `/orbvi/depth` | `sensor_msgs/Image` | Generated depth image |
| `/orbvi/depth/viz` | `sensor_msgs/Image` | Depth visualization image |
| `/orbvi/depth/panorama` | `sensor_msgs/Image` | `32FC1` spherical range derived from four disparity tiles |
| `/orbvi/depth/panorama/viz` | `sensor_msgs/Image` | Depth-panorama visualization |
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

ROS2 Foxy or Jazzy:

```bash
export ROS_DISTRO=foxy  # or jazzy
sudo apt update
sudo apt install -y \
  cmake libboost-dev libopencv-dev \
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

Choose one launch file. Each user-facing launch accepts only `host`.

Visual data, depth and VIO:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch host:=<device-ip>
ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py host:=<device-ip>
```

Visual data, depth, VIO and MID360:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch.py host:=<device-ip>
```

Visual data, depth, VIO and Host-derived panorama:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_pano.launch host:=<device-ip>
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_pano.launch.py host:=<device-ip>
```

Public topic names are fixed under `/orbvi`. The panorama launch uses the
bundled `config/pano_mode2.yaml` / `config/pano_mode2_ros2.yaml` defaults. It
projects a 2048x1024 angular canvas, crops 200 rows at the top and bottom, and
publishes 2048x624 fixed-seam, four-level MultiBand output using the calibrated
fisheye rotations and translations with a configurable 2.0 m finite stitching
radius. The physical mapping is fixed as `front=3`, `right=2`,
`rear=4`, `left=5`. Depth assistance is disabled by default because disparity
from the stereo network trails the raw fisheye capture; using stale geometry on
moving objects would increase ghosting. A throttled log reports the effective
camera mapping and whether the rotation extrinsics are applied.

The optional depth path retains the latest four `RigDepthPanorama` products and
selects the closest one without waiting. Enable it only after RGB/disparity
capture timestamps have been aligned and verified.

Depth assistance requires the matching Host SDK ABI. Always refresh both
`third_party/orbvi_sdk/include/orbvi_sdk` and the architecture-specific
`liborbvi_sdk`, then clean and rebuild the Bridge. Replacing only the `.so` is
not supported.

ROS2 launch files set the bundled Fast DDS shared-memory profile for the bridge
process. Start standalone ROS2 tools with the same profile when they need to
consume large local topics:

```bash
SHM_PROFILE="$(ros2 pkg prefix orbvi_ros_bridge)/share/orbvi_ros_bridge/config/fastdds_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
ros2 daemon stop
```

## Visual And MID360 Data Check

Start the MID360 entry.

ROS1:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
```

ROS2:

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch.py host:=<device-ip>
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

Outputs are written to a timestamped directory:

- `summary.log`
- `rates.csv`
- `topics/*.hz.log`

## Launch Files

The public interface is intentionally limited to three launch configurations.
Each launch accepts only `host`; stream selection and reference settings are
kept inside the launch file.

| Capability | ROS1 launch | ROS2 launch | First topics to check |
| --- | --- | --- | --- |
| Visual, depth and VIO | `orbvi_ros_bridge.launch` | `orbvi_ros_bridge.launch.py` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry` |
| Visual, depth, VIO and MID360 | `orbvi_ros_bridge_mid360.launch` | `orbvi_ros_bridge_mid360.launch.py` | `/orbvi/lidar/custom`, `/orbvi/lidar/imu` |
| Visual, depth, VIO and panorama | `orbvi_ros_bridge_pano.launch` | `orbvi_ros_bridge_pano.launch.py` | `/orbvi/pano/image` |

| Parameter | Default | Description |
| --- | --- | --- |
| `host` | `127.0.0.1` | ORBVI device IP or hostname |

Topic names are fixed under `/orbvi`.

## View Data

ROS1 images:

```bash
rosrun rqt_image_view rqt_image_view /orbvi/raw/camera_0/image
```

ROS2 images:

```bash
ros2 run rqt_image_view rqt_image_view /orbvi/raw/camera_0/image
```

Point clouds and odometry can be viewed in RViz:

- `/orbvi/lidar/custom`
- `/orbvi/depth/points`
- `/orbvi/vio/odometry`

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
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip>
```

Then check `/orbvi/disparity`, `/orbvi/depth` and the bridge log.

## License

MIT
