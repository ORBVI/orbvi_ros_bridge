# View Data

[中文版本](VIEWING.zh-CN.md) · [Back to README](../README.md)

## RViz

The default ROS1/ROS2 RViz layout docks four raw cameras in one row at the top:
front (3), right (2), rear (4), left (5), with the point-cloud view below.
These directions refer to the device; use TF for robot mounting directions.
Reopen the default RViz launch to load the layout.

The ROS1/ROS2 RViz configs and launch files default to Fixed Frame
`camera_init`, the 4VL Omni-LIVO world, so depth, LiDAR and the trajectory are
shown together in the world while the device moves. This requires the bridge's
dynamic TF from `camera_init` (`publish_vio_tf`, on by default; child `aft_mapped`, or `base_link` in robot mode) and the
calibrated sensor TF tree; start the 4VL bridge launch with
`publish_lidar_pcl:=true` to include LiDAR points. Without a localization TF,
pass another frame, e.g. `fixed_frame:=orbvi_link`:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_rviz.launch fixed_frame:=orbvi_link
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_rviz.launch.py fixed_frame:=orbvi_link
```

### Fixed Frame choices

| Frame | Meaning | Use it to view |
| --- | --- | --- |
| `camera_init` (default) | Omni-LIVO world fixed at initialization: gravity-aligned Z-up, origin at the initial MID360 IMU, yaw not tied to the robot front | 4VL trajectory, map/scan clouds, and depth/LiDAR placed in the world |
| `odom` | Optional startup origin above `camera_init` (`tf_mode:=robot publish_odom_tf:=true`): origin on the ground below the robot, yaw along the initial robot front | Robot motion starting from zero |
| `world` | 4V VINS world | 4V trajectory only; not connected to the depth tree yet |
| `aft_mapped` / `mid360_imu` | Current MID360 IMU state (moves with the device); `mid360_imu` is the same physical frame | IMU-centred view of the device |
| `orbvi_link` | Device-fixed SDK ROS_BODY reference: camera-rig origin, axes forward/left/up | Depth in device coordinates, depth/LiDAR alignment; needs no localization TF |
| `base_link` | Robot body (robot mode, from URDF or `mounting_config`) | Data in robot coordinates |
| `livox_frame` | MID360 point coordinates | Raw LiDAR alone |
| `orbvi/camera/<id>` | Raw camera optical frames (right/down/forward), IDs 3/2/4/5 = front/right/rear/left | Checking a single camera's projection |

Frames move with the device except `camera_init`, `odom` and `world`. With a
device-fixed frame the device stays still and the world moves; with a world
frame the device moves. See [TF](TF.md) and
[robot integration](ROBOT_INTEGRATION.md) for the full trees.

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

The ROS2 bridge, RViz and rqt launch files inherit `RMW_IMPLEMENTATION`,
`FASTRTPS_DEFAULT_PROFILES_FILE`, `FASTDDS_DEFAULT_PROFILES_FILE` and
`RMW_FASTRTPS_PUBLICATION_MODE`, filling project defaults only when a variable
is unset. If only one profile path variable is set, the other uses that path;
if both are set, each retains its value. `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`,
`ROS_AUTOMATIC_DISCOVERY_RANGE` and `ROS_STATIC_PEERS` are inherited directly.

Without external configuration, the defaults use Fast DDS and the bundled
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

For local use, set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and clear remote
`ROS_STATIC_PEERS` in the shared startup environment before starting the
bridge, viewers and tools. Existing `ROS_LOCALHOST_ONLY=1` configurations
are also honored in Jazzy. The project does not fix the domain ID or read
`.zshrc` itself; services, IDEs and scripts must pass the environment to their
child processes. Changing variables in another terminal does not update
running nodes.

First confirm discovery with `ros2 node list --no-daemon`, then check actual
delivery with `ros2 topic hz /orbvi/imu`. A running process or a listed topic
does not prove that messages are arriving. Do not delete `/dev/shm/fastrtps*`
while nodes are running; exit the processes using those files before cleaning
up stale resources.
