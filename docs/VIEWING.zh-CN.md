# 查看数据

[English](VIEWING.md) · [返回 README](../README.zh-CN.md)

## RViz

ROS1/ROS2 默认 RViz 布局将四路原始相机固定在顶部一排，顺序为
前（3）、右（2）、后（4）、左（5），下方为点云视图。方向名称指设备本身方向，
机器人安装后的方向以 TF 为准。重新打开默认 RViz launch 即可加载该布局。

ROS1/ROS2 RViz launch 默认使用 `profile:=mid360`，固定坐标系为 `camera_init`，
启用 `/orbvi/cloud_registered` 的 1000 秒累积显示。使用 `profile:=visual` 时，
在 `orbvi_link` 中显示实时彩色深度点云，不依赖定位 TF；
需要实时单帧时可单独打开 `Registered RGB Cloud (Live)`。
`fixed_frame:=<frame>` 可覆盖任一预设的坐标系。

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_rviz.launch profile:=mid360
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_rviz.launch.py profile:=mid360
```

累积显示保留每帧扫描，不做体素去重；内存和渲染成本随点数、频率及保留时长增长。
需要减少资源占用时，降低 RViz 的 Decay Time，或关闭累积显示并启用 Live。
查看原始雷达时，在 4VL bridge launch 设置 `publish_lidar_pcl:=true`，打开
`LiDAR Points`；单独查看可设置 `fixed_frame:=livox_frame`。

`mid360` 世界系视图依赖 bridge 从 `camera_init` 出发的动态 TF
（`publish_localization_tf`，默认开启；子坐标系为 `aft_mapped`，机器人模式为
`base_link`）和完整标定的传感器 TF。没有定位 TF 时使用 `profile:=visual`
或 `fixed_frame:=orbvi_link`。

### Fixed Frame 选择

| 坐标系 | 含义 | 适合查看 |
| --- | --- | --- |
| `camera_init`（`mid360` 默认） | Omni-LIVO 初始化时固定的世界系：重力对齐、Z 轴向上，原点在初始化时的 MID360 IMU，yaw 不保证朝机器人前方 | 4VL 轨迹、scan/map 点云，以及放到世界中的深度/雷达 |
| `odom` | `camera_init` 之上的可选启动原点（`tf_mode:=robot publish_odom_tf:=true`）：原点为机器人下方地面投影，yaw 对齐初始机器人前方 | 从零开始的机器人运动 |
| `world` | 4V VINS 世界系 | 仅 4V 轨迹；目前与深度点云的树不连通 |
| `aft_mapped` / `mid360_imu` | 当前 MID360 IMU 状态系（随设备运动）；`mid360_imu` 是同一物理坐标系 | 以 IMU 为中心观察设备 |
| `orbvi_link`（`visual` 默认） | 设备固连的 SDK ROS_BODY 参考系：原点为相机 rig 参考原点，轴向前/左/上 | 设备系下的深度点云、深度与雷达对齐检查；不依赖定位 TF |
| `base_link` | 机器人本体系（机器人模式，由 URDF 或 `mounting_config` 提供） | 机器人坐标下的数据 |
| `livox_frame` | MID360 点云坐标系 | 单独查看原始雷达 |
| `orbvi/camera/<id>` | 原始相机光学系（右/下/前），ID 3/2/4/5 对应前/右/后/左 | 检查单路相机投影 |

除 `camera_init`、`odom`、`world` 外，其余坐标系都随设备运动：选设备固连系时设备
不动、世界在动；选世界系时设备在动。完整 TF 树见 [TF](TF.md) 与
[机器人安装与 TF](ROBOT_INTEGRATION.md)。

点云、里程计和轨迹可在 RViz 中查看：

- `/orbvi/lidar/custom`
- `/orbvi/depth/points`
- `/orbvi/localization/odometry`
- `/orbvi/localization/path`

仓库自带的 ROS1/ROS2 RViz 配置已预置 `Localization Path` display。若轨迹存在但不可见，
将 RViz 的 `Fixed Frame` 设置为 `/orbvi/localization/odometry.header.frame_id` 的实际值
（4V VINS 通常为 `world`，4VL Omni-LIVO 为 `camera_init`）。
4V 当前发布的 `world → body` 与深度点云的 `orbvi_link → cameras` 是两棵树，
bridge 尚未通过 VINS 相机—IMU 外参连接它们；因此切到 `world` 可看轨迹，
切到 `orbvi_link` 可看点云，联合显示还需要这段标定 TF，不能用单位变换代替。

## 图像

Bridge 运行后，用自带 launch 启动 rqt：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_rqt.launch
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_rqt.launch.py
```

ROS2 launch 会设置与 bridge launch 相同的 Fast DDS 共享内存配置，无需手动导出环境变量。
launch 打开 rqt 主窗口，从 `Plugins` 菜单添加插件：

- `Topics → Topic Monitor`：列出 topic、类型、频率和带宽，勾选 topic 后开始测量，
  可替代 `ros2 topic list/type/hz/bw`。
- `Visualization → Image View`：选择 `/orbvi/raw/camera_0/image`、`/orbvi/pano/image`、
  `/orbvi/ipm/image`、`/orbvi/depth/panorama/viz` 等 topic，可同时打开多个面板。

rqt 会保存布局，下次启动时自动恢复。缺少插件时先安装，例如
`sudo apt install ros-$ROS_DISTRO-rqt-topic ros-$ROS_DISTRO-rqt-image-view`。

## ROS2 传输与发现

ROS2 的 bridge、RViz 和 rqt launch 优先继承启动环境中的 `RMW_IMPLEMENTATION`、
`FASTRTPS_DEFAULT_PROFILES_FILE`、`FASTDDS_DEFAULT_PROFILES_FILE` 和
`RMW_FASTRTPS_PUBLICATION_MODE`，仅在变量未设置时补充项目默认值。只设置一个
profile 路径变量时，另一个沿用该路径；两个都设置时分别保留原值。
`ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、`ROS_AUTOMATIC_DISCOVERY_RANGE` 和
`ROS_STATIC_PEERS` 直接继承，不由 launch 覆盖。

没有外部配置时，默认使用 Fast DDS 与内置 profile：UDPv4 用于
节点发现，同机且启用 SHM 的参与者通过共享内存传输业务数据。这样普通 ROS2
工具也能发现 bridge，无需为查询 topic 而加载专用 profile。
不要强制仅使用 SHM：它与默认使用 UDP 发现的参与者混用时可能互相不可见，详见
[Fast DDS 传输说明](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/shared_memory/shared_memory.html)。

读取本机大流量 topic 时，仍建议为工具导出相同配置，以使用相同的共享内存资源预算：

```bash
SHM_PROFILE="$(ros2 pkg prefix orbvi_ros_bridge)/share/orbvi_ros_bridge/config/fastdds_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
ros2 daemon stop
```

Bridge、可视化和工具应使用相同的 `ROS_DOMAIN_ID`。更换 Domain、RMW 或 profile 后，
重启相关进程并停止旧的 `ros2 daemon`，避免查询沿用旧环境。配置更新对已经运行的
进程不生效；从旧的仅 SHM 配置迁移时，应同时重启 bridge 和可视化。

仅需本机使用时，在统一的启动环境中设置 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
并清除远端 `ROS_STATIC_PEERS`，再从该环境启动 bridge、RViz、rqt 和工具。已有的
`ROS_LOCALHOST_ONLY=1` 在 Jazzy 中仍可使用。项目不固定域 ID，也不会主动读取
`.zshrc`；通过服务、IDE 或脚本启动时，需要由启动者传入这些环境变量。
仅在另一个终端修改变量，不会更新已运行的节点。

排查时先用 `ros2 node list --no-daemon` 确认发现 bridge，再用
`ros2 topic hz /orbvi/imu` 确认实际数据。进程存活或 topic 名称存在不能代替收帧验证。
不要在节点运行时删除 `/dev/shm/fastrtps*`；需要清理残留时，先退出使用它们的进程。
