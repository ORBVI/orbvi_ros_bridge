# 查看数据

[English](VIEWING.md) · [返回 README](../README.zh-CN.md)

## RViz

ROS1/ROS2 默认 RViz 布局将四路原始相机固定在顶部一排，顺序为
前（3）、右（2）、后（4）、左（5），下方为点云视图。方向名称指设备本身方向，
机器人安装后的方向以 TF 为准。重新打开默认 RViz launch 即可加载该布局。

ROS2 自带 RViz 配置默认启用 `Color Depth Points`、`LiDAR Points` 和 TF 显示，
固定坐标系为 `orbvi_link`，直接显示设备坐标系下的深度点云，不依赖定位 TF。
使用 4VL bridge launch 并传入 `publish_lidar_pcl:=true`，在完整标定 TF 下可同时
查看深度点云、雷达和轨迹。单独查看原始雷达时可传入 `fixed_frame:=livox_frame`。

点云、里程计和轨迹可在 RViz 中查看：

- `/orbvi/lidar/custom`
- `/orbvi/depth/points`
- `/orbvi/vio/odometry`
- `/orbvi/vio/path`

仓库自带的 ROS1/ROS2 RViz 配置已预置 `VIO Path` display。若轨迹存在但不可见，
将 RViz 的 `Fixed Frame` 设置为 `/orbvi/vio/odometry.header.frame_id` 的实际值
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

ROS2 的 bridge、RViz 和 rqt launch 都会设置内置 Fast DDS profile：UDPv4 用于
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

排查时先用 `ros2 node list --no-daemon` 确认发现 bridge，再用
`ros2 topic hz /orbvi/imu` 确认实际数据。进程存活或 topic 名称存在不能代替收帧验证。
不要在节点运行时删除 `/dev/shm/fastrtps*`；需要清理残留时，先退出使用它们的进程。
