# 检查与排障

[English](TROUBLESHOOTING.md) · [返回 README](../README.zh-CN.md)

## 视觉和 MID360 数据检查

启动 4VL 入口：

ROS1:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch
```

ROS2:

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch.py
```

在 bridge 主机检查 topic 是否存在及类型：

```bash
rostopic list | grep '^/orbvi/lidar'
rostopic type /orbvi/lidar/custom
rostopic type /orbvi/lidar/imu
```

测量 ROS1 频率：

```bash
rostopic hz /orbvi/lidar/custom --window 30
rostopic hz /orbvi/lidar/imu --window 100
```

ROS2 在另一个终端用自带 launch 启动 rqt。它使用与 bridge 相同的共享内存配置，
无需手动导出环境变量：

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_rqt.launch.py
```

添加 `Plugins → Topics → Topic Monitor`，勾选 `/orbvi/lidar/custom` 和
`/orbvi/lidar/imu` 查看类型与频率。详见 [查看数据](VIEWING.zh-CN.md)。

健康状态下 `/orbvi/lidar/custom` 通常约 10 Hz，`/orbvi/lidar/imu` 通常约 200 Hz。
具体数值会受 MID360 配置、网络质量和主机负载影响。

## 全 topic 频率检查

脚本从 ROS bridge 主机的本地 ROS master 检查 topic，应在 source 了 bridge workspace 的
同一台机器上运行。以下命令在仓库根目录执行。

MID360 topic 子集：

```bash
scripts/orbvi_ros_bridge_rate_check.sh \
  --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
  --topics /orbvi/lidar/custom,/orbvi/lidar/imu \
  --sample-sec 20
```

全部 `/orbvi/*` topic：

```bash
scripts/orbvi_ros_bridge_rate_check.sh \
  --bridge-setup ~/orbvi_ros1_ws/devel/setup.bash \
  --sample-sec 20
```

输出会写入带时间戳的目录：

- `summary.log`
- `rates.csv`
- `topics/*.hz.log`

## FAQ

### 没有 topic 发布

确认 ROS 和 bridge workspace 在同一个 shell 中 source：

```bash
source /opt/ros/noetic/setup.bash
source ~/orbvi_ros1_ws/devel/setup.bash
rosnode list
```

从 bridge 主机检查设备控制端口：

```bash
nc -vz <device-ip> 18088
```

### 无法加载 `livox_ros_driver2/CustomMsg`

确认整个仓库 clone 到 workspace 的 `src` 目录，并重新构建 workspace。本仓库包含
`CustomMsg` 所需的 message-only `livox_ros_driver2` 包，不包含 Livox 驱动节点或设备配置工具。

### MID360 topic 存在但频率为空

先启动 4VL launch，再从同一 bridge 主机测量：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch
rostopic hz /orbvi/lidar/custom --window 30
```

如果类型存在但没有样本，保持 bridge 运行，并用设备管理工具检查设备侧 MID360 状态。

### 缺少深度 topic

深度输出依赖 Host SDK 的视差帧和有效标定：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch
```

然后检查 `/orbvi/disparity`、`/orbvi/depth` 和 bridge 日志。
