# View Data

[中文版本](VIEWING.zh-CN.md) · [Back to README](../README.md)

## RViz

The default ROS1/ROS2 RViz layout docks four raw cameras in one row at the top:
front (3), right (2), rear (4), left (5), with the point-cloud view below.
These directions refer to the device; use TF for robot mounting directions.
Reopen the default RViz launch to load the layout.

The ROS2 RViz config enables `Color Depth Points`, `LiDAR Points` and TF in
`orbvi_link` by default, showing device-frame depth without a localization TF.
Start the 4VL bridge launch with `publish_lidar_pcl:=true` to view depth, LiDAR
and poses together when the calibrated TF tree is complete. Pass
`fixed_frame:=livox_frame` to view raw LiDAR independently.

Point clouds, odometry, and the trajectory can be viewed in RViz:

- `/orbvi/lidar/custom`
- `/orbvi/depth/points`
- `/orbvi/vio/odometry`
- `/orbvi/vio/path`

The bundled ROS1 and ROS2 RViz configurations include a `VIO Path` display.
If the topic is active but the trajectory is not visible, set RViz `Fixed
Frame` to the actual `/orbvi/vio/odometry.header.frame_id` value (normally
`world` for 4V VINS and `camera_init` for 4VL Omni-LIVO).
For 4V, the current `world → body` and depth `orbvi_link → cameras` branches
are separate trees: the bridge does not yet connect them using VINS camera/IMU
extrinsics. Use `world` for the trajectory or `orbvi_link` for depth. Showing
both together requires that calibrated TF; an identity transform is not a substitute.

## Images

Start rqt from the bundled launch after the bridge is running:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_rqt.launch
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_rqt.launch.py
```

The ROS2 launch applies the same Fast DDS shared-memory settings as the bridge
launch, so no environment needs to be exported by hand. The launch opens the rqt
main window; add plugins from the `Plugins` menu:

- `Topics → Topic Monitor`: topic list, type, rate and bandwidth. Tick a topic
  to start measuring it; this replaces `ros2 topic list/type/hz/bw`.
- `Visualization → Image View`: choose `/orbvi/raw/camera_0/image`,
  `/orbvi/pano/image`, `/orbvi/ipm/image`, `/orbvi/depth/panorama/viz` and
  similar topics. Several Image View panels can be open at once.

rqt saves the layout and restores it on the next start. If a plugin is missing,
install it, for example `sudo apt install ros-$ROS_DISTRO-rqt-topic
ros-$ROS_DISTRO-rqt-image-view`.

## ROS2 Transport and Discovery

The ROS2 bridge, RViz and rqt launch files all set the bundled Fast DDS
profile: UDPv4 handles discovery, while local participants with SHM enabled
exchange user data through shared memory. Ordinary ROS2 tools can therefore
discover the bridge without loading a custom profile just to inspect topics.
Avoid SHM-only discovery: mixing it with participants using default UDP
discovery can make them invisible to one another. See the
[Fast DDS transport documentation](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/shared_memory/shared_memory.html).

For large local topics, exporting the same profile is still recommended so
tools use the same shared-memory resource budget:

```bash
SHM_PROFILE="$(ros2 pkg prefix orbvi_ros_bridge)/share/orbvi_ros_bridge/config/fastdds_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
ros2 daemon stop
```

Use the same `ROS_DOMAIN_ID` for the bridge, viewers and tools. After changing
the Domain, RMW or profile, restart affected processes and stop the old
`ros2 daemon` so queries do not retain its previous environment. Running
processes do not reload profile changes; migrating from the old SHM-only
profile requires restarting both the bridge and viewers.

First confirm discovery with `ros2 node list --no-daemon`, then check actual
delivery with `ros2 topic hz /orbvi/imu`. A running process or a listed topic
does not prove that messages are arriving. Do not delete `/dev/shm/fastrtps*`
while nodes are running; exit the processes using those files before cleaning
up stale resources.
