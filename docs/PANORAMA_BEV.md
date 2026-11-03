# Panorama And BEV

[中文版本](PANORAMA_BEV.zh-CN.md) · [Back to README](../README.md)

## BEV/IPM

Both 4V and 4VL enable the board-preferred four-fisheye 360-degree BEV/IPM by
default through the `bev` token in `streams`. `publish_ipm:=true` turns on the
same output, which is useful only when `streams` is overridden without `bev`:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch \
  streams:=raw,imu,vio publish_ipm:=true
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4v.launch.py \
  streams:=raw,imu,vio publish_ipm:=true
```

The Bridge first subscribes the board-generated `bev_display_stream` and only
decodes its JPEG into a `1000x1000` `bgr8` ROS image. The board target rate is
20 Hz. If the device explicitly reports that stream as unsupported, the Host
fallback assumes four horizontal cameras at 1.0 m above the ground and generates
a 20 m x 20 m, 0.02 m/pixel, 1000x1000 BGR image at up to 5 Hz. Timeouts, blocked
streams and runtime failures do not activate Host derivation.
This describes default device mode. For robot-mounted ground projection using
full installation extrinsics, see [robot TF and IPM](ROBOT_INTEGRATION.md).

The `ipm_*` arguments are listed in [Launch Arguments](LAUNCH_ARGUMENTS.md).

## Panorama

Both 4V and 4VL request the board-preferred panorama by default:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4v.launch.py
```

Public topic names are fixed under `/orbvi`. Both launches first request
the board `pano_display_stream` and `bev_display_stream`. Only a board response
of `UnsupportedStream` enables the matching Host-derived fallback. The bundled
`config/pano_mode2.yaml` / `config/pano_mode2_ros2.yaml` geometry is used as the
expected board panorama layout and as the complete Host fallback policy. The
Host fallback projects a 2048x1024 angular canvas, crops 200 rows at the top and
bottom, and publishes 2048x624 fixed-seam, four-level MultiBand output. The
physical mapping is fixed as `front=3`, `right=2`,
`rear=4`, `left=5`. The Host SDK normalizes RGB and rig depth panoramas to the
same `[0, 2*pi)` azimuth contract, so the Bridge copies both images without
phase adjustment. The public `/orbvi/depth/panorama` image uses a cached
output-pixel inverse lookup built from the rectified intrinsics and rotations.
It intentionally does not apply stereo-pair translations, matching the original
clean visualization path. The separate optional RGB depth-assistance path keeps
using `RigDepthPanorama` with full rotation and translation. Neither path uses
point-cloud generation. Depth assistance is disabled by default because disparity
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
