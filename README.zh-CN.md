# ORBVI ROS Bridge

[English](README.md)

首次使用随仓库提供的 4VL 板端 deb，请阅读
[安装与使用说明](third_party/orbvi_sdk/deb/README.md)：包含校验、板端安装、
版本检查、主机 bridge 启动和回滚方法。

ORBVI ROS Bridge 通过 ORBVI Host SDK 将 ORBVI 设备数据发布到 ROS1 或 ROS2。
它面向需要在 Linux 主机上获取相机、IMU、MID360 LiDAR、视差、Host 侧深度、
板端全景/BEV 和 VIO topic 的应用开发者。

仓库包含：

- ROS1 Noetic 和 ROS2 Foxy/Jazzy bridge 节点
- 视觉、MID360、全景三类 launch 文件
- bridge 所需的最小 `livox_ros_driver2` CustomMsg 定义
- MID360 和全 topic 频率检查工具

## 支持平台

| 平台 | 系统 | ROS | 状态 |
| --- | --- | --- | --- |
| x86_64 / amd64 工作站 | Ubuntu 20.04 | ROS1 Noetic | 支持 |
| x86_64 / amd64 工作站 | Ubuntu 20.04 | ROS2 Foxy | 支持 |
| x86_64 / amd64 工作站 | Ubuntu 24.04 | ROS2 Jazzy | 已测试 |
| aarch64 / arm64 ORBVI 主机 | Ubuntu 20.04 | ROS1 Noetic | 支持 |
| aarch64 / arm64 ORBVI 主机 | Ubuntu 20.04 | ROS2 Foxy | 支持 |

## Topic 约定

ROS1 和 ROS2 使用相同 topic 名称。ROS2 消息类型使用对应的 `::msg` 命名空间。

校正图 topic 的方向段读取 SDK Core 的 `pair_direction`：同一个 `pair_id` 的
`left`、`right` 必须使用相同方向。`source_camera_direction` 继续表示物理相机
来源，仅用于标定和诊断，不参与 topic 选择。

| Topic | ROS1 类型 | 说明 |
| --- | --- | --- |
| `/orbvi/raw/camera_<id>/image` | `sensor_msgs/Image` | 开启 decoded 时发布的 BGR 图像 |
| `/orbvi/raw/camera_<id>/image/compressed` | `sensor_msgs/CompressedImage` | 原始鱼眼压缩图像 transport |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image` | `sensor_msgs/Image` | 开启 decoded 时发布的校正图像 |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image/compressed` | `sensor_msgs/CompressedImage` | 校正后的 stereo pair 压缩图像 transport |
| `/orbvi/pano/image` | `sensor_msgs/Image` | 优先发布板端 `pano_display_stream`；仅板端明确不支持时才由 Host 派生 |
| `/orbvi/ipm/image` | `sensor_msgs/Image` | 优先发布板端 `bev_display_stream`（`1000x1000`、`bgr8`、目标 20 Hz）；仅板端明确不支持时才由 Host 派生 |
| `/orbvi/imu` | `sensor_msgs/Imu` | 设备 IMU |
| `/orbvi/lidar/custom` | `livox_ros_driver2/CustomMsg` | MID360 点云流 |
| `/orbvi/lidar/imu` | `sensor_msgs/Imu` | MID360 IMU 流 |
| `/orbvi/disparity` | `sensor_msgs/Image` | 视差图 |
| `/orbvi/depth` | `sensor_msgs/Image` | 生成的深度图 |
| `/orbvi/<front\|right\|rear\|left>/depth/viz` | `sensor_msgs/Image` | 四方向深度可视化图，可选叠加 5×5 米制深度网格 |
| `/orbvi/depth/panorama` | `sensor_msgs/Image` | 由四路视差生成的 `32FC1` 球面距离图 |
| `/orbvi/depth/panorama/viz` | `sensor_msgs/Image` | 全景深度可视化图 |
| `/orbvi/depth/points` | `sensor_msgs/PointCloud2` | 深度生成的点云 |
| `/orbvi/vio/odometry` | `nav_msgs/Odometry` | 优化后的 VIO 位姿 |
| `/orbvi/vio/path` | `nav_msgs/Path` | 由优化后的 VIO 位姿累计得到的有界轨迹 |
| `/orbvi/vio/imu_prediction` | `nav_msgs/Odometry` | IMU 预测的 VIO 位姿 |

矫正图中的方位表示双目对的观察方向，不表示原始鱼眼相机本身。八路图像固定按
`front/left、front/right、right/left、right/right、rear/left、rear/right、left/left、left/right`
发布，物理像素来源如下：

| 双目方位 | 左目来源 | 右目来源 |
| --- | --- | --- |
| `front` | front 鱼眼右半部分 | right 鱼眼左半部分 |
| `right` | right 鱼眼右半部分 | rear 鱼眼左半部分 |
| `rear` | rear 鱼眼右半部分 | left 鱼眼左半部分 |
| `left` | left 鱼眼右半部分 | front 鱼眼左半部分 |

## 安装依赖

在 bridge 主机上安装 ROS 和构建工具。

ROS1 Noetic:

```bash
sudo apt update
sudo apt install -y \
  cmake libboost-dev libyaml-cpp-dev libopencv-dev \
  ros-noetic-roscpp \
  ros-noetic-std-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-message-generation
```

ROS2 Foxy 或 Jazzy:

```bash
export ROS_DISTRO=foxy  # 或 jazzy
sudo apt update
sudo apt install -y \
  cmake libboost-dev libyaml-cpp-dev libopencv-dev \
  python3-colcon-common-extensions \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-nav-msgs \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-rosidl-default-generators
```

仓库已经包含 `orbvi_ros_bridge` 所需的 `livox_ros_driver2` 消息定义，fresh clone 到
ROS workspace 的 `src` 目录后即可直接编译。

## 构建 ROS1

```bash
mkdir -p ~/orbvi_ros1_ws/src
cd ~/orbvi_ros1_ws/src
git clone <this-repo-url> orbvi_ros_bridge
cd ..

source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```


## 构建 ROS2

```bash
mkdir -p ~/orbvi_ros2_ws/src
cd ~/orbvi_ros2_ws/src
git clone <this-repo-url> orbvi_ros_bridge
cd ..

source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动 Bridge

根据需要选择一个 launch 文件。三套 launch 的公共参数见下文。

视觉数据、深度和 VIO：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch host:=<device-ip>
ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py host:=<device-ip>
```

启用板端优先的四路鱼眼 360° BEV/IPM：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> publish_ipm:=true
ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py \
  host:=<device-ip> publish_ipm:=true
```

Bridge 先订阅板端 `bev_display_stream`，Host 只把板端 JPEG 解码为 `1000x1000`
`bgr8` ROS Image，板端目标频率为 20 Hz。仅设备明确返回该流不支持时才启用
Host 回退：默认假设四个相机离地 1.0 m，覆盖车体周围 20 m × 20 m，以
0.02 m/px 生成 1000x1000 BGR，最高 5 Hz。超时、流被阻塞或运行时故障都不会
触发板外派生。
以上为默认设备模式；使用完整安装外参和机器人地面平面的投影请见
[机器人 TF 与 IPM](docs/ROBOT_INTEGRATION.md)。

如需在四方向深度可视化图中绘制 5×5 网格，并显示每个网格交点的深度值，启用
`depth_viz_show_grid_values`：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> depth_viz_show_grid_values:=true
ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py \
  host:=<device-ip> depth_viz_show_grid_values:=true
```

有效深度保留两位小数并带 `m` 单位；无效或超出显示范围的采样点显示为 `--`。
该参数只修改 `/orbvi/<front|right|rear|left>/depth/viz` 四路可视化图，原始深度图和
点云输出不受影响。叠加开销固定为每方向 5 条横线、5 条竖线和 25 个文本标签，直接
在现有可视化缓冲区原位绘制；参数关闭时完全跳过这部分处理。

视觉数据、深度、VIO 和 MID360：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch.py host:=<device-ip>
```

视觉数据、深度、VIO、板端优先全景和板端优先 BEV：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_pano.launch host:=<device-ip>
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_pano.launch.py host:=<device-ip>
```

公开 topic 固定在 `/orbvi` 下。全景入口先请求板端 `pano_display_stream` 和
`bev_display_stream`；只有板端返回 `UnsupportedStream` 才启用对应 Host 回退。
内置 `config/pano_mode2.yaml` / `config/pano_mode2_ros2.yaml` 既定义板端全景预期
几何，也定义完整 Host 回退策略。Host 回退以 2048x1024 完整角度画布投影，
上下各裁 200 行后发布 2048x624 图像。物理相机映射固定为
`front=3、right=2、rear=4、left=5`。Host SDK 已将 RGB 与 rig 深度全景统一为
`[0, 2*pi)` 方位契约，Bridge 仅复制图像，不再执行相位调整。
公开 `/orbvi/depth/panorama` 恢复为按输出像素反向查表：查表仅在标定加载时构建一次，
使用 rectified 内参与旋转外参，刻意不应用各双目光心平移，以保持最早版本的连续显示
风格。独立的 RGB 深度辅助仍使用带完整旋转和平移的 `RigDepthPanorama`。两条链路均不
经过点云生成接口。
双目网络输出的 disparity 天然晚于原始鱼眼帧，默认配置暂时关闭深度辅助，避免把
旧深度用于运动目标；运行日志会打印实际相机映射及旋转外参是否生效。
可选深度路径会缓存最近 4 个 `RigDepthPanorama`，RGB 回调只无阻塞读取时间最接近
的缓存。只有完成 RGB/disparity 原始采集时刻对齐并验证后，才应重新开启该路径。

深度辅助依赖新版 Host SDK ABI。更新 Bridge 时必须同时更新
`third_party/orbvi_sdk/include/orbvi_sdk` 和对应架构的 `liborbvi_sdk`，然后清理
`build/devel` 重新编译；不能只替换 `.so`。

ROS2 launch 会为 bridge 进程设置内置 Fast DDS 共享内存 profile。单独启动的
`ros2 topic`、rqt、rviz 或其它消费进程如需消费本机大流量 topic，使用相同环境：

```bash
SHM_PROFILE="$(ros2 pkg prefix orbvi_ros_bridge)/share/orbvi_ros_bridge/config/fastdds_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
ros2 daemon stop
```

## 视觉和 MID360 数据检查

启动 MID360 入口：

ROS1:

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
```

ROS2:

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch.py host:=<device-ip>
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

健康状态下 `/orbvi/lidar/custom` 通常约 10 Hz，`/orbvi/lidar/imu` 通常约 200 Hz。
具体数值会受 MID360 配置、网络质量和主机负载影响。

## 全 topic 频率检查

脚本从 ROS bridge 主机的本地 ROS master 检查 topic，应在 source 了 bridge workspace 的
同一台机器上运行。

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

## Launch 文件

对外接口固定为三套 launch 配置；每套 launch 提供对应工作流的 `streams` 默认值，
调用方可以覆盖。

| 能力 | ROS1 launch | ROS2 launch | 首要检查 topic |
| --- | --- | --- | --- |
| 视觉、深度和 VIO | `orbvi_ros_bridge.launch` | `orbvi_ros_bridge.launch.py` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry` |
| 视觉、深度、VIO 和 MID360 | `orbvi_ros_bridge_mid360.launch` | `orbvi_ros_bridge_mid360.launch.py` | `/orbvi/lidar/custom`, `/orbvi/lidar/imu` |
| 视觉、深度、VIO、全景和 BEV | `orbvi_ros_bridge_pano.launch` | `orbvi_ros_bridge_pano.launch.py` | `/orbvi/pano/image`, `/orbvi/ipm/image` |

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `127.0.0.1` | ORBVI 设备 IP 或 hostname |
| `streams` | 随 launch 变化 | 逗号分隔的 stream token；`pano`、`bev` 表示板端优先显示流 |
| `publish_lidar_pcl` | `false` | 额外将 Livox `CustomMsg` 转换并发布到 `/orbvi/lidar/points` |
| `publish_ipm` | `false` | 请求 `/orbvi/ipm/image`：设备模式优先板端显示流，机器人模式读取板端 GPU 地面投影，不回退 CPU |
| `tf_mode` | `device` | `robot` 模式转换机器人位姿并应用安装外参 |
| `mounting_config` | 空 | 推荐 `sdk` 读取板端唯一安装/地面快照；兼容 URDF frame 配置或本地 xyz/rpy YAML |
| `publish_mounting_tf` | `false` | 选择安装广播者：URDF 或 bridge；`mounting_config:=sdk` 时数值来自板端 GPU 快照 |
| `publish_odom_tf` | `false` | 机器人模式发布启动 odom → camera_init，归零 footprint 和航向；需 native Z-up 与地面平面 |
| `odom_frame` | `odom` | 可选启动锚点的父 frame |
| `depth_pointcloud_frame_id` | 空（ROS_BODY 默认 `orbvi_link`） | 同步设置 ROS_BODY 点云和静态 TF 的设备参考系名称；`base_link` 兼容旧版 |
| `ipm_camera_height_m` | `1.0` | 设备模式 Host 回退高度；机器人模式使用完整外参和地面平面 |
| `ipm_extent_m` | `10.0` | 仅设备模式 CPU 回退覆盖范围，米；GPU 由板端配置 |
| `ipm_resolution_m_per_pixel` | `0.02` | 仅设备模式 CPU 回退米/像素，上限 4096；板端 GPU 默认 1000、上限 1024 |
| `ipm_max_publish_rate_hz` | `5.0` | 仅设备模式 CPU 回退最大发布频率；0 表示不节流 |
| `ipm_frame_id` | 自动 | 设备模式为 `base_link`；机器人模式为 mounting parent，不允许另设不一致的 frame |
| `depth_viz_show_grid_values` | `false` | 在每路方向深度可视化图上绘制 5×5 网格及网格交点米制深度值 |
| `publish_vio_path` | `true` | 将优化后的 `/orbvi/vio/odometry` 累计并发布到 `/orbvi/vio/path` |
| `vio_path_max_poses` | `2000` | Path 中保留的最大 Pose 数；小于等于 0 时按 1 处理 |
| `vio_path_publish_rate_hz` | `10.0` | Path 最大发布频率；设为 0 时随每条优化 odometry 发布 |

Topic 名称固定在 `/orbvi` 下。

`/orbvi/vio/path` 不使用 `/orbvi/vio/imu_prediction`。每个 Pose 沿用优化 odometry
的坐标系、时间戳和位姿；坐标系变化或时间戳回退时自动清空旧轨迹。默认最多保留
2000 个 Pose，内存和单条消息大小均为 `O(vio_path_max_poses)`，整条 Path 默认最多
发布 10 Hz。高频或低带宽场景可调小点数或频率上限，或设置
`publish_vio_path:=false` 关闭。

4VL 的板端 Omni-LIVO 使用同一位姿流，bridge 通过
`localization_source=omni_livo` 和 `pose_kind=OPTIMIZED` 识别优化位姿，
并发布到 `/orbvi/vio/odometry` 和 `/orbvi/vio/path`。

ROS1/ROS2 bridge 均支持 Omni-LIVO 动态 TF 与标定静态 TF：
默认设备模式下，`camera_init → aft_mapped` 连接雷达、雷达 IMU、四路原始相机及 SDK ROS_BODY 深度点云。
静态 TF 在标定加载后立即发布，不等待位姿。`publish_vio_tf` 默认开启；
`publish_sensor_tf` 默认继承它，也可单独开启。新增 `orbvi_export_extrinsics`
导出带标定 hash 的外参 JSON。原始相机 frame 统一为 `orbvi/camera/<SDK camera ID>`，
雷达 IMU frame 修正为 `mid360_imu`；topic 名称与传感器数值保持原契约。
设备参考系和 ROS_BODY 深度点云现在默认使用 `orbvi_link`（X 前、Y 左、Z 上），
四路原始相机为光学系（X 右、Y 下、Z 前）。如需兼容旧名称，设置
`depth_pointcloud_frame_id:=base_link`，导出时同步使用 `--device-frame base_link`。
此参数不改变 IPM/BEV 的 header，也不提供机器人安装或地面外参。
机器人模式单独配置安装外参，发布 `camera_init → base_link → orbvi_link → 传感器`，
并读取板端 GPU 生成的机器人坐标系 IPM，核对安装 TF、地面平面和标定 hash，支持正装、倒装和旋转安装。
URDF 模式通过 TF 读取安装外参，不再另读 YAML 安装数值；发现重复发布或变化时
暂停机器人输出。可选 `publish_odom_tf:=true` 将起始 footprint 和航向对齐到 odom。
配置、迁移和启动示例见 [机器人 TF 与 IPM](docs/ROBOT_INTEGRATION.md)。
坐标定义、缺标定行为、兼容说明和验收命令见 [TF 与外参](docs/TF.md)。

ROS2 传感器 topic 默认使用 Reliable QoS，兼容 RViz 的 Reliable 和 Best Effort
订阅。需要 Best Effort 发布时，给 bridge launch 传入
`sensor_data_best_effort:=true`；此时消费端也必须使用 Best Effort。

## 查看数据

ROS1/ROS2 默认 RViz 布局将四路原始相机固定在顶部一排，顺序为
前（3）、右（2）、后（4）、左（5），下方为点云视图。方向名称指设备本身方向，
机器人安装后的方向以 TF 为准。重新打开默认 RViz launch 即可加载该布局。

ROS2 自带 RViz 配置默认启用 `Color Depth Points`、`LiDAR Points` 和 TF 显示，
固定坐标系为 `camera_init`。使用 MID360 bridge launch 并传入
`publish_lidar_pcl:=true` 时可同时查看彩色深度点云、雷达和轨迹。没有定位流时，传入
`fixed_frame:=orbvi_link` 查看深度点云，或 `fixed_frame:=livox_frame` 查看原始雷达。

ROS1 图像：

```bash
rosrun rqt_image_view rqt_image_view /orbvi/raw/camera_0/image
```

ROS2 图像：

```bash
ros2 run rqt_image_view rqt_image_view /orbvi/raw/camera_0/image
```

点云、里程计和轨迹可在 RViz 中查看：

- `/orbvi/lidar/custom`
- `/orbvi/depth/points`
- `/orbvi/vio/odometry`
- `/orbvi/vio/path`

仓库自带的 ROS1/ROS2 RViz 配置已预置 `VIO Path` display。若轨迹存在但不可见，
将 RViz 的 `Fixed Frame` 设置为 `/orbvi/vio/odometry.header.frame_id` 的实际值
（设备当前通常为 `world`）。

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

先启动 MID360 launch，再从同一 bridge 主机测量：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>
rostopic hz /orbvi/lidar/custom --window 30
```

如果类型存在但没有样本，保持 bridge 运行，并用设备管理工具检查设备侧 MID360 状态。

### 缺少深度 topic

深度输出依赖 Host SDK 的视差帧和有效标定：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip>
```

然后检查 `/orbvi/disparity`、`/orbvi/depth` 和 bridge 日志。

## License

MIT
