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
- 4V、4VL 两套 bridge launch，以及 RViz、rqt 查看 launch
- bridge 所需的最小 `livox_ros_driver2` CustomMsg 定义
- MID360 和全 topic 频率检查工具

## 支持平台

| 平台 | 系统 | ROS | 状态 |
| --- | --- | --- | --- |
| x86_64 / amd64 工作站 | Ubuntu 20.04 | ROS1 Noetic | 支持 |
| x86_64 / amd64 工作站 | Ubuntu 20.04 | ROS2 Foxy | 支持 |
| x86_64 / amd64 工作站 | Ubuntu 24.04 | ROS2 Jazzy | 支持 |
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

按设备型号选择 launch 文件，两个 launch 默认都开启该型号的全部能力。
需要精简输出时覆盖 `streams`。公共参数见 [Launch 参数](docs/LAUNCH_ARGUMENTS.zh-CN.md)。

设备地址默认为 `192.168.3.100`。设备使用其他地址时，在下面的命令后追加
`host:=<device-ip>`。

| 设备 | ROS1 launch | ROS2 launch | 默认 `streams` |
| --- | --- | --- | --- |
| 4V（四目，无激光雷达） | `orbvi_ros_bridge_4v.launch` | `orbvi_ros_bridge_4v.launch.py` | `raw,rectified,pano,bev,imu,disparity,depth,vio` |
| 4VL（四目，带激光雷达） | `orbvi_ros_bridge_4vl.launch` | `orbvi_ros_bridge_4vl.launch.py` | `raw,rectified,pano,bev,imu,lidar,lidar_imu,disparity,depth,vio` |

4V：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4v.launch.py
```

4VL：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch.py
```

首要检查的 topic：`/orbvi/raw/camera_<id>/image`、`/orbvi/depth`、
`/orbvi/vio/odometry`、`/orbvi/pano/image`、`/orbvi/ipm/image`；4VL 另有
`/orbvi/lidar/custom` 和 `/orbvi/lidar/imu`。

新固件的全景和 BEV 均由板端生成，主机只做解码发布；仅旧固件明确返回不支持时
才回退到主机端派生。不需要时可从 `streams` 中去掉 `pano,bev`，例如：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch \
  streams:=raw,rectified,imu,disparity,depth,vio
```

## 文档

| 主题 | 文档 |
| --- | --- |
| Launch 参数、深度网格叠加、VIO 轨迹、位姿/TF 概要、ROS2 QoS | [Launch 参数](docs/LAUNCH_ARGUMENTS.zh-CN.md) |
| BEV/IPM 与全景行为、Host 回退、深度辅助 | [全景与 BEV](docs/PANORAMA_BEV.zh-CN.md) |
| RViz、rqt（topic 频率与图像）、ROS2 共享内存传输 | [查看数据](docs/VIEWING.zh-CN.md) |
| MID360 数据检查、全 topic 频率检查、FAQ | [检查与排障](docs/TROUBLESHOOTING.zh-CN.md) |
| 坐标定义与外参标定 | [TF 与外参](docs/TF.md) |
| 机器人安装、机器人 TF 与地面 IPM | [机器人 TF 与 IPM](docs/ROBOT_INTEGRATION.md) |

## License

MIT
