# ORBVI ROS Bridge

[English](README.md)

ORBVI ROS Bridge 通过 ORBVI Host SDK 将 ORBVI 设备数据发布到 ROS1 或 ROS2。
它面向需要在 Linux 主机上获取相机、IMU、MID360 LiDAR、视差、Host 侧深度、
Host 侧全景和 VIO topic 的应用开发者。

仓库包含：

- ROS1 Noetic 和 ROS2 Foxy/Jazzy bridge 节点
- 全数据、视觉、视觉 + MID360 三类 launch 文件
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
| `/orbvi/depth/viz` | `sensor_msgs/Image` | 深度可视化图 |
| `/orbvi/depth/points` | `sensor_msgs/PointCloud2` | 深度生成的点云 |
| `/orbvi/vio/odometry` | `nav_msgs/Odometry` | 优化后的 VIO 位姿 |
| `/orbvi/vio/imu_prediction` | `nav_msgs/Odometry` | IMU 预测的 VIO 位姿 |

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

ROS1 全流：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip>
```

可选 Host 侧全景：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> streams:=raw,rectified,pano,imu,lidar,lidar_imu,disparity,depth,vio
```

默认全景配置为 `config/pano_mode2.yaml`，使用 Host SDK blend mode 2
（`pano_blend=multiband`）和 `pano_seam_blend_px=32`。

ROS2 全流：

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py \
  host:=<device-ip>
```

可选 Host 侧全景：

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py \
  host:=<device-ip> streams:=raw,rectified,pano,imu,lidar,lidar_imu,disparity,depth,vio
```

ROS2 默认全景配置为 `config/pano_mode2_ros2.yaml`，同样使用 mode 2
和 32 px 接缝融合配置。

## 全景配置

`pano` 是 Host SDK 基于 `raw_fisheye_stream` 拼接出的全景图。
bridge 不依赖设备端输出 `pano_display_stream`。

Launch 文件默认加载内置全景配置：

| ROS | 配置文件 | 格式 |
| --- | --- | --- |
| ROS1 | `config/pano_mode2.yaml` | 扁平 ROS 参数 YAML |
| ROS2 | `config/pano_mode2_ros2.yaml` | ROS2 节点参数 YAML |

两个配置都使用当前参考默认值：

```yaml
pano_blend: "multiband"
pano_seam_blend_px: 32
pano_width: 2048
pano_height: 1024
pano_seam_mode: "fixed"
pano_photometric_align: true
```

需要替换配置时传 `pano_config`：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=<device-ip> \
  streams:=raw,rectified,pano,imu,lidar,lidar_imu,disparity,depth,vio \
  pano_config:=/path/to/pano_mode2.yaml

ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py \
  host:=<device-ip> \
  streams:=raw,rectified,pano,imu,lidar,lidar_imu,disparity,depth,vio \
  pano_config:=/path/to/pano_mode2_ros2.yaml
```

## 场景速查

根据运行需求选择对应 launch。公开 topic 固定在 `/orbvi` 下。

| 场景 | ROS1 命令 | ROS2 命令 | 首要检查 |
| --- | --- | --- | --- |
| 基础全数据 bridge | `roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py host:=<device-ip>` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry`, `/orbvi/lidar/custom` |
| 相机、深度和 VIO | `roslaunch orbvi_ros_bridge orbvi_ros_bridge_visual.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge orbvi_ros_bridge_visual.launch.py host:=<device-ip>` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry` |
| 相机、深度、VIO 和 MID360 | `roslaunch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch host:=<device-ip>` | `ros2 launch orbvi_ros_bridge orbvi_ros_bridge_mid360.launch.py host:=<device-ip>` | `/orbvi/raw/camera_<id>/image`, `/orbvi/depth`, `/orbvi/vio/odometry`, `/orbvi/lidar/custom`, `/orbvi/lidar/imu` |

只需要视觉原始数据、深度和 VIO 时使用 `orbvi_ros_bridge_visual.launch`。
需要视觉能力和 MID360 点云、MID360 IMU 同时运行时使用
`orbvi_ros_bridge_mid360.launch`。


ROS2 launch 默认会在 bridge 主机上启用 Fast DDS 共享内存：

```bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py \
  host:=<device-ip>
```

launch 默认会为 bridge 进程设置 Fast DDS SHM 环境。
但单独启动的 `ros2 topic`、rqt、rviz 或其它消费进程也必须带相同环境，才能加入同一个
本机 SHM transport：

```bash
SHM_PROFILE="$(ros2 pkg prefix orbvi_ros_bridge)/share/orbvi_ros_bridge/config/fastdds_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$SHM_PROFILE"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
ros2 daemon stop
```

## 视觉和 MID360 数据检查

启动视觉 + MID360：

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

Launch 文件按 ROS 版本和运行场景分组。

| Launch 文件 | ROS | 用途 |
| --- | --- | --- |
| `orbvi_ros_bridge.launch` | ROS1 | 默认全原始数据入口：视觉数据、深度、VIO 和 MID360 |
| `orbvi_ros_bridge_visual.launch` | ROS1 | 视觉入口：四路 raw decoded 图、rectified 图、设备 IMU、视差、深度和 VIO |
| `orbvi_ros_bridge_mid360.launch` | ROS1 | 视觉入口 + MID360 点云和 MID360 IMU |
| `orbvi_ros_bridge.launch.py` | ROS2 | 默认全原始数据入口：视觉数据、深度、VIO 和 MID360 |
| `orbvi_ros_bridge_visual.launch.py` | ROS2 | 视觉入口：四路 raw decoded 图、rectified 图、设备 IMU、视差、深度和 VIO |
| `orbvi_ros_bridge_mid360.launch.py` | ROS2 | 视觉入口 + MID360 点云和 MID360 IMU |

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `127.0.0.1` | ORBVI 设备 IP 或 hostname |
| `control_port` | `18088` | ORBVI SDK 控制端口 |
| `connect_timeout_ms` | `2000` | 首次 Host SDK 连接每次尝试的超时时间 |
| `connect_retry_count` | `4` | 首次连接失败后的重试次数 |
| `connect_retry_delay_ms` | `1000` | Host SDK 连接重试间隔 |
| `pano_config` | 内置 YAML | launch 级全景配置文件，默认使用 mode 2 / 32 px 融合配置 |
| `pano_profile` | `baseline` | 全景测试档位：`baseline`、`ghost_suppression`/`balanced` 或 `primary_only` |
| `pano_width` | `2048` | `streams` 包含 `pano` 时 Host SDK 全景输出宽度 |
| `pano_height` | `1024` | `streams` 包含 `pano` 时 Host SDK 全景输出高度 |
| `pano_fov_half_deg` | `95.0` | Host SDK 全景拼接使用的水平半视场角 |
| `pano_seam_blend_px` | `32` | Host SDK 全景拼接接缝融合宽度 |
| `pano_seam_mode` | `fixed` | Host SDK 全景接缝模式：`fixed` 或 `dynamic`/`dynamic_programming`/`dp` |
| `pano_dp_seam_band_px` | `96` | 动态规划接缝搜索带宽，单位像素 |
| `pano_dp_seam_smoothness` | `8.0` | 动态规划接缝搜索使用的平滑惩罚 |
| `pano_seam_avoidance_penalty` | `220.0` | 启用动态接缝选择时，接缝避让 mask 使用的惩罚值 |
| `pano_blend` | `multiband` | Host SDK 全景融合模式：`multiband`/`mode2`、`feather` 或 `primary_only` |
| `pano_photometric_align` | `true` | 是否启用 Host SDK 全景逐帧曝光对齐 |
| `pano_seam_ghost_suppression` | `false` | 是否启用 Host SDK 接缝局部重影抑制 |
| `pano_seam_ghost_threshold` | `80.0` | 接缝重影抑制使用的颜色/亮度差异阈值 |
| `use_fastdds_shm` | `true` | ROS2 bridge 进程是否启用内置 Fast DDS SHM profile |
| `fastdds_shm_profile` | 内置 XML | ROS2 Fast DDS SHM profile 路径，可用自定义 profile 覆盖 |

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
roslaunch orbvi_ros_bridge orbvi_ros_bridge_visual.launch \
  host:=<device-ip>
```

然后检查 `/orbvi/disparity`、`/orbvi/depth` 和 bridge 日志。

## License

MIT
