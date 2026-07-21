# ORBVI ROS Bridge

[English](README.md)

ORBVI ROS Bridge 通过 ORBVI Host SDK 将 ORBVI 设备数据发布到 ROS1 或 ROS2。
它面向需要在 Linux 主机上获取相机、IMU、MID360 LiDAR、视差、Host 侧深度、
Host 侧全景和 VIO topic 的应用开发者。

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

| Topic | ROS1 类型 | 说明 |
| --- | --- | --- |
| `/orbvi/raw/camera_<id>/image` | `sensor_msgs/Image` | 开启 decoded 时发布的 BGR 图像 |
| `/orbvi/raw/camera_<id>/image/compressed` | `sensor_msgs/CompressedImage` | 原始鱼眼压缩图像 transport |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image` | `sensor_msgs/Image` | 开启 decoded 时发布的校正图像 |
| `/orbvi/rectified/<front\|right\|rear\|left>/<left\|right>/image/compressed` | `sensor_msgs/CompressedImage` | 校正后的 stereo pair 压缩图像 transport |
| `/orbvi/pano/image` | `sensor_msgs/Image` | `streams` 包含 `pano` 时，由 Host SDK 从 `raw_fisheye_stream` 拼接出的全景图 |
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
  cmake libboost-dev libopencv-dev \
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
  cmake libboost-dev libopencv-dev \
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

视觉数据、深度、VIO 和 Host 侧全景：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_pano.launch host:=<device-ip>
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_pano.launch.py host:=<device-ip>
```

公开 topic 固定在 `/orbvi` 下。全景入口使用内置 `config/pano_mode2.yaml` /
`config/pano_mode2_ros2.yaml` 默认配置。默认以 2048x1024 完整角度画布投影，
上下各裁 200 行后发布 2048x624 图像；默认使用固定接缝、4 层 MultiBand，以及
鱼眼旋转/平移外参和可调的 2.0 m 有限拼接半径。物理相机映射固定为
`front=3、right=2、rear=4、left=5`。
RGB SDK 输出原本采用 `[-pi, pi)` 方位范围，而深度全景采用 `[0, 2*pi)`；默认配置
`pano_align_to_depth: true` 会在发布时将 RGB 无插值循环平移半幅，使
`/orbvi/pano/image` 与 `/orbvi/depth/panorama` 的同一横坐标表示同一方位。设为
`false` 可恢复 Host SDK 的原始 RGB 布局。
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

对外接口固定为三套 launch 配置；stream 组合和参考配置保留在 launch 文件内部。

| 能力 | ROS1 launch | ROS2 launch | 首要检查 topic |
| --- | --- | --- | --- |
| 视觉、深度和 VIO | `orbvi_ros_bridge.launch` | `orbvi_ros_bridge.launch.py` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry` |
| 视觉、深度、VIO 和 MID360 | `orbvi_ros_bridge_mid360.launch` | `orbvi_ros_bridge_mid360.launch.py` | `/orbvi/lidar/custom`, `/orbvi/lidar/imu` |
| 视觉、深度、VIO 和全景 | `orbvi_ros_bridge_pano.launch` | `orbvi_ros_bridge_pano.launch.py` | `/orbvi/pano/image` |

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `127.0.0.1` | ORBVI 设备 IP 或 hostname |
| `publish_lidar_pcl` | `false` | 额外将 Livox `CustomMsg` 转换并发布到 `/orbvi/lidar/points` |
| `depth_viz_show_grid_values` | `false` | 在每路方向深度可视化图上绘制 5×5 网格及网格交点米制深度值 |

Topic 名称固定在 `/orbvi` 下。

## 查看数据

ROS1 图像：

```bash
rosrun rqt_image_view rqt_image_view /orbvi/raw/camera_0/image
```

ROS2 图像：

```bash
ros2 run rqt_image_view rqt_image_view /orbvi/raw/camera_0/image
```

点云和里程计可在 RViz 中查看：

- `/orbvi/lidar/custom`
- `/orbvi/depth/points`
- `/orbvi/vio/odometry`

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
