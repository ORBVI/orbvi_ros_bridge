# Launch Arguments

[中文版本](LAUNCH_ARGUMENTS.zh-CN.md) · [Back to README](../README.md)

The public interface is intentionally limited to two launch configurations,
4V and 4VL; see [Start The Bridge](../README.md#start-the-bridge). Each one
enables every capability of its device model by default, and callers may
override `streams`. Both accept the common arguments below.

## Common Arguments

| Parameter | Default | Description |
| --- | --- | --- |
| `host` | `192.168.3.100` | ORBVI device IP or hostname |
| `streams` | 4V: `raw,rectified,pano,bev,imu,disparity,depth,localization`; 4VL: `raw,rectified,pano,bev,imu,lidar,lidar_imu,disparity,depth,localization,registered_cloud` | Comma-separated stream tokens; `pano` and `bev` select board-preferred display outputs |
| `publish_lidar_pcl` | `false` | Also convert Livox `CustomMsg` to `/orbvi/lidar/points` |
| `publish_ipm` | `false` | Request `/orbvi/ipm/image`: board-preferred in device mode, board GPU robot-ground projection without CPU fallback in robot mode |
| `tf_mode` | `device` | Use `robot` to publish robot pose and apply installation extrinsics |
| `mounting_config` | empty | Use `sdk` for the board mounting/ground snapshot; local URDF/frame or xyz/rpy YAML remains supported |
| `publish_mounting_tf` | `false` | Choose installation publisher: URDF or bridge; `mounting_config:=sdk` reads values from the board GPU snapshot |
| `publish_odom_tf` | `false` | Robot mode: anchor startup footprint and heading with static odom → camera_init; requires native Z-up and ground plane |
| `odom_frame` | `odom` | Parent frame of the optional startup anchor |
| `depth_pointcloud_frame_id` | empty (ROS_BODY: `orbvi_link`) | Device reference used by ROS_BODY cloud headers and static TF; `base_link` restores legacy naming |
| `ipm_camera_height_m` | `1.0` | Device-mode Host fallback height; robot mode uses full extrinsics and ground plane |
| `ipm_extent_m` | `10.0` | Device-mode CPU fallback coverage, metres; GPU geometry is configured on the board |
| `ipm_resolution_m_per_pixel` | `0.02` | Device-mode CPU fallback metres per pixel, cap 4096; board GPU default 1000, cap 1024 |
| `ipm_max_publish_rate_hz` | `5.0` | Device-mode CPU fallback maximum rate; zero disables throttling |
| `ipm_frame_id` | auto | Device: `base_link`; robot: mounting parent, which cannot be overridden with a different frame |
| `depth_viz_show_grid_values` | `false` | Draw a 5x5 grid and metric grid-point values on each directional depth visualization |
| `publish_localization_path` | `true` | Accumulate optimized `/orbvi/localization/odometry` and publish `/orbvi/localization/path` |
| `localization_path_max_poses` | `2000` | Maximum poses retained in the Path; values at or below zero are clamped to 1 |
| `localization_path_publish_rate_hz` | `10.0` | Maximum Path publication rate; zero publishes every optimized odometry update |

Topic names are fixed under `/orbvi`. BEV/IPM and panorama behavior is described
in [Panorama and BEV](PANORAMA_BEV.md).

## Localization Migration and Stream Compatibility

Pose, trajectory and prediction use `/orbvi/localization/{odometry,path,imu_prediction}`.
Update legacy subscribers or use ROS remapping. Launch arguments use
`publish_localization_tf`, `publish_localization_path`, `localization_path_max_poses`
and `localization_path_publish_rate_hz`. The node honors deprecated `publish_vio_*`
and `vio_path_*` parameters, with a warning, only when the new name is unset;
launch entrypoints accept the new names. `localization`, `vio` and the legacy
`vio_pose_stream` token subscribe to both `optimized_pose_stream` and
`predicted_pose_stream`; the deprecated mixed stream is never subscribed.
Either split stream name can also be requested individually. Optimized poses
publish odometry, path and dynamic TF; predictions publish only
`/orbvi/localization/imu_prediction`.

LiDAR acceleration is converted to m/s² using SDK unit parameters; covariance
is multiplied by the squared scale, retaining the `-1` unavailable marker.
Startup and conversion share supported-unit and finite-scale checks. Missing or
invalid parameters skip only `lidar_imu` with a warning; units are never guessed.

4VL requests optional `registered_cloud` by default. A manifest explicitly
excluding it causes a skip; legacy firmware without a manifest is probed by
subscription. Primary streams subscribe before optional registered clouds.
Individual subscription failures preserve other streams; startup fails if no
subscriptions succeed. Async admission without a first frame after
`first_frame_timeout_ms` produces a per-stream warning with the SDK reason.
Restart the bridge after fixing board capability or configuration.

Rectified JPEG output supports legacy bundles, V2 dual atlases and UVC/Bulk
single atlases through SDK lossless cropping into eight images. Invalid
topic/frame identities, empty images and unsupported formats produce diagnostics.

ROS2 workers serialize each topic while independent topics run concurrently;
enqueue wakes one worker. Overflow drops the oldest pending task with a
throttled warning. Source sampling timestamps remain unchanged; this does not
repair board clock regressions.

## Depth Visualization Grid

To draw a 5x5 grid and the depth at every grid intersection on all four
directional depth visualizations, enable `depth_viz_show_grid_values`:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch \
  depth_viz_show_grid_values:=true
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4v.launch.py \
  depth_viz_show_grid_values:=true
```

Valid values are rendered with two decimal places and the `m` suffix; invalid
or out-of-range samples are rendered as `--`. The option changes only the four
`/orbvi/<front|right|rear|left>/depth/viz` images. Raw depth and point-cloud
outputs are unchanged. Overlay work is bounded to five horizontal lines, five
vertical lines, and 25 labels per direction; drawing is in-place on the
existing visualization buffer and is skipped entirely while the option is
disabled.

## Localization Path, Pose And TF

`/orbvi/localization/path` does not consume `/orbvi/localization/imu_prediction`. Each pose keeps
the optimized odometry frame, timestamp, and pose. The history resets when the
frame changes or timestamps roll back. Memory and message size are bounded to
`O(localization_path_max_poses)`, and the complete Path is published at no more than
10 Hz by default. Reduce the pose or rate limit for high-rate or low-bandwidth
links, or set `publish_localization_path:=false` to disable the output.

The 4VL native Omni-LIVO adapter uses the same pose stream. Both ROS1 and ROS2
recognize `localization_source=omni_livo`: `pose_kind=OPTIMIZED` is published on
`/orbvi/localization/odometry` and `/orbvi/localization/path`, while `pose_kind=PROPAGATED` is
published on `/orbvi/localization/imu_prediction`. Predictions never broadcast optimized
TF. Optimized samples marked `tracking_state=DEGRADED` (including `recovering`)
remain available as Odometry but do not advance TF until `TRACKING` resumes.
A TF rejection message reports the actual tracking reason, timestamp,
sequence, frame or installation-calibration failure. The first rejection warns
immediately; subsequent summaries are limited to one report per 60 seconds,
using a monotonic host clock. Summaries count accepted/rejected optimized poses.
After at least five seconds of uninterrupted acceptance, the next eligible
report announces recovery at INFO level. Brief recovery/rejection oscillations
do not restart the log interval. Algorithm degradation is
not an unsupported SDK pose format; inspect `tracking_reason` on the device.
See the [pose metadata validation](validation/2026-10-21-pose-metadata.md)
for the tested versions, live results and remaining depth-quality limits.

Both ROS1 and ROS2 bridges publish Omni-LIVO dynamic and calibrated static TF.
In default device mode `camera_init → aft_mapped` connects LiDAR, LiDAR IMU, four raw cameras and SDK
ROS_BODY depth clouds. Static TF is available immediately after calibration,
without a pose subscription. `publish_localization_tf` defaults to true;
`publish_sensor_tf` inherits it and can be enabled independently. The standalone
`orbvi_export_extrinsics` tool exports JSON with calibration hashes. Raw image
frames become `orbvi/camera/<SDK camera ID>`; LiDAR IMU becomes `mid360_imu`.

The device reference and ROS_BODY depth clouds default to `orbvi_link`
(forward, left, up); raw cameras use optical axes (right, down, forward).
For legacy names, set `depth_pointcloud_frame_id:=base_link` and export with
`--device-frame base_link`. IPM/BEV headers remain separately configured.
Robot mode separates mounting from internal calibration and publishes
`camera_init → base_link → orbvi_link → sensors`. It also computes robot-frame
IPM from the board GPU with audited mounting/ground/calibration snapshots, including inverted and rotated
installations. URDF mode reads the actual static TF without a second mounting pose
in YAML; duplicate/changing installation TF pauses robot outputs. Optional
`publish_odom_tf:=true` anchors startup footprint and heading in `odom`. See
[robot TF and IPM setup](ROBOT_INTEGRATION.md).
Topic names and sensor values retain their existing contract. See
[TF and extrinsic calibration](TF.md) for frame definitions, migration,
missing-calibration behavior and verification commands.

## ROS2 QoS

ROS2 sensor topics default to Reliable QoS, supporting both Reliable and
Best Effort subscribers. Set `sensor_data_best_effort:=true` on the bridge
launch to use Best Effort publishers; subscribers must then use Best Effort.
