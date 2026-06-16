# ORBVI ROS Bridge

ORBVI ROS Bridge publishes ORBVI device data into ROS1 or ROS2 through the
ORBVI Host SDK. It is the open bridge package for application developers who
need camera, IMU, MID360 LiDAR, disparity, depth and VIO topics on Linux hosts.

The repository includes:

- ROS1 Noetic and ROS2 Foxy bridge nodes
- Launch files for all streams, MID360-only and visual/VIO-only workflows
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

The bridge follows a product SDK layout: users clone one repository, build one
ROS package and get a working bridge without rebuilding the Host SDK.

- `third_party/orbvi_sdk/include/orbvi_sdk` contains the public Host SDK C++
  headers used by both ROS1 and ROS2.
- `third_party/orbvi_sdk/libs/linux-x86_64` contains the Ubuntu x86_64 SDK
  libraries.
- `third_party/orbvi_sdk/libs/linux-aarch64` contains the Ubuntu arm64 SDK
  libraries for ORBVI hosts.
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
  ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs
```

ROS2 Foxy:

```bash
sudo apt update
sudo apt install -y \
  cmake libboost-dev libopencv-dev \
  python3-colcon-common-extensions \
  ros-foxy-rclcpp \
  ros-foxy-sensor-msgs \
  ros-foxy-nav-msgs
```

`livox_ros_driver2` message definitions must be available in the same workspace.
The CI fixture under `ci/livox_ros_driver2_msgs` is only for automated build
validation; production workspaces should use the real Livox package.

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

For lower CPU load, publish compressed images only:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> \
  image_mode:=raw-only \
  queue_size:=2 \
  max_receive_queue_depth:=4 \
  max_decode_queue_depth:=4
```

ROS2 uses the same parameter names:

```bash
ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge.launch.py \
  host:=<device-ip> \
  image_mode:=raw-only \
  queue_size:=2
```

## Scenario Quickstart

Use these entry points first, then tune parameters only after the expected
topics appear.

| Scenario | ROS1 command | ROS2 command | Expected first check |
| --- | --- | --- | --- |
| Basic all-data bridge | `roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge.launch.py host:=<device-ip>` | `/orbvi/imu`, `/orbvi/raw/*`, `/orbvi/lidar/*` |
| MID360 only | `roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge_mid360.launch.py host:=<device-ip>` | `/orbvi/lidar/custom`, `/orbvi/lidar/imu` |
| Camera, depth and VIO | `roslaunch orbvi_ros_bridge orbvi_ros_bridge_visual.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge ros2/orbvi_ros_bridge_visual.launch.py host:=<device-ip>` | `/orbvi/depth`, `/orbvi/depth/points`, `/orbvi/vio/odometry` |

This mirrors the common SDK workflow: start the smallest useful launch file,
inspect the expected topics, then enable heavier decoded image/depth outputs
only when the host has enough CPU headroom.

## MID360 Data Check

Start only MID360 streams.

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

MID360-only:

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
  --streams lidar,lidar_imu \
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
| `orbvi_ros_bridge.launch` | ROS1 | All stream entry |
| `orbvi_ros_bridge_mid360.launch` | ROS1 | MID360 LiDAR + MID360 IMU |
| `orbvi_ros_bridge_visual.launch` | ROS1 | Camera + device IMU + depth + VIO |
| `ros2/orbvi_ros_bridge.launch.py` | ROS2 | All stream entry |
| `ros2/orbvi_ros_bridge_mid360.launch.py` | ROS2 | MID360 LiDAR + MID360 IMU |
| `ros2/orbvi_ros_bridge_visual.launch.py` | ROS2 | Camera + device IMU + depth + VIO |

Common parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `host` | `127.0.0.1` | ORBVI device IP or hostname |
| `control_port` | `18088` | ORBVI SDK control port |
| `streams` | `all` | Comma-separated stream list, or `all` |
| `topic_prefix` | `/orbvi` | ROS topic prefix |
| `image_mode` | `raw-and-decoded` | `raw-only`, `decoded` or `raw-and-decoded` |
| `queue_size` | `4` | ROS publisher queue size |
| `max_receive_queue_depth` | `8` | Host SDK receive queue depth |
| `max_decode_queue_depth` | `8` | Host SDK decode queue depth |
| `publish_depth` | `auto` | `auto`, `true` or `false` |
| `publish_depth_viz` | `true` | Publish `/orbvi/depth/viz` |
| `publish_depth_pointcloud` | `true` | Publish `/orbvi/depth/points` |

Less common tuning remains available as private node parameters. For example,
set `depth_format`, `depth_pointcloud_stride`, `max_decode_latency_ms` or
`allow_sample_endpoint_fallback` from your own launch wrapper when needed.

Stream aliases:

| Alias | Host SDK stream |
| --- | --- |
| `raw` | Raw fisheye |
| `rectified` | Rectified fisheye |
| `imu` | Device IMU |
| `lidar` | MID360 point cloud |
| `lidar_imu` | MID360 IMU |
| `disparity` | Disparity image |
| `depth` | Generated depth output, requires disparity |
| `vio` | VIO pose |

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

Maintainer CI validates ROS1 Noetic and ROS2 Foxy builds on both x86_64 and
arm64 Linux hosts. Normal pipelines only compile and test the current ROS
packages with the prebuilt CI images selected by `CI_IMAGE_TAG_SUFFIX`. Docker
image refresh jobs are manual maintenance jobs and should only be run when the
CI Dockerfile or base dependency set changes.

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

Build the real `livox_ros_driver2` message package in the same workspace, then
source the workspace again. The CI fixture is intentionally minimal and should
not replace the production Livox package.

### MID360 topic exists but rate is empty

Run the MID360 launch first, then measure from the same bridge host:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
rostopic hz /orbvi/lidar/custom --window 30
```

If the type exists but no samples arrive, keep the bridge running and inspect
device-side MID360 state with device management tooling.

### Image decoding uses too much CPU

Use compressed-only output:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> \
  image_mode:=raw-only
```

### Depth topics are missing

Depth output depends on disparity frames and valid calibration from the Host
SDK:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_visual.launch \
  host:=<device-ip> \
  publish_depth:=true
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
