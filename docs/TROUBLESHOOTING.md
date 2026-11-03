# Checks And Troubleshooting

[中文版本](TROUBLESHOOTING.zh-CN.md) · [Back to README](../README.md)

## Visual And MID360 Data Check

Start the 4VL entry.

ROS1:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch
```

ROS2:

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch.py
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

For ROS2, start rqt from the bundled launch in another terminal. It uses the
same shared-memory settings as the bridge, so no environment export is needed:

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_rqt.launch.py
```

Add `Plugins → Topics → Topic Monitor` and tick `/orbvi/lidar/custom` and
`/orbvi/lidar/imu` to see their type and rate. See [View Data](VIEWING.md) for
details.

Typical healthy values are around 10 Hz for `/orbvi/lidar/custom` and around
200 Hz for `/orbvi/lidar/imu`. Exact values depend on MID360 configuration,
network quality and host load.

## Full Topic Rate Check

The script checks topics from the ROS bridge host's local ROS master. It should
run on the same machine that sources the ROS bridge workspace. Run the commands
from the repository root.

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

Run the 4VL launch first, then measure from the same bridge host:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch
rostopic hz /orbvi/lidar/custom --window 30
```

If the type exists but no samples arrive, keep the bridge running and inspect
device-side MID360 state with device management tooling.

### Depth topics are missing

Depth output depends on disparity frames and valid calibration from the Host
SDK:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch
```

Then check `/orbvi/disparity`, `/orbvi/depth` and the bridge log.
