# Launch 参数

[English](LAUNCH_ARGUMENTS.md) · [返回 README](../README.zh-CN.md)

对外接口固定为 4V 和 4VL 两套 launch 配置，见 [启动 Bridge](../README.zh-CN.md#启动-bridge)。
两套 launch 默认开启对应型号的全部能力，调用方可以覆盖 `streams`。公共参数如下。

## 公共参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `192.168.3.100` | ORBVI 设备 IP 或 hostname |
| `streams` | 4V：`raw,rectified,pano,bev,imu,disparity,depth,vio`；4VL：`raw,rectified,pano,bev,imu,lidar,lidar_imu,disparity,depth,vio` | 逗号分隔的 stream token；`pano`、`bev` 表示板端优先显示流 |
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

Topic 名称固定在 `/orbvi` 下。BEV/IPM 与全景行为见 [全景与 BEV](PANORAMA_BEV.zh-CN.md)。

## 深度可视化网格

如需在四方向深度可视化图中绘制 5×5 网格，并显示每个网格交点的深度值，启用
`depth_viz_show_grid_values`：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch \
  depth_viz_show_grid_values:=true
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4v.launch.py \
  depth_viz_show_grid_values:=true
```

有效深度保留两位小数并带 `m` 单位；无效或超出显示范围的采样点显示为 `--`。
该参数只修改 `/orbvi/<front|right|rear|left>/depth/viz` 四路可视化图，原始深度图和
点云输出不受影响。叠加开销固定为每方向 5 条横线、5 条竖线和 25 个文本标签，直接
在现有可视化缓冲区原位绘制；参数关闭时完全跳过这部分处理。

## VIO 轨迹、位姿与 TF

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
配置、迁移和启动示例见 [机器人 TF 与 IPM](ROBOT_INTEGRATION.md)。
坐标定义、缺标定行为、兼容说明和验收命令见 [TF 与外参](TF.md)。

## ROS2 QoS

ROS2 传感器 topic 默认使用 Reliable QoS，兼容 RViz 的 Reliable 和 Best Effort
订阅。需要 Best Effort 发布时，给 bridge launch 传入
`sensor_data_best_effort:=true`；此时消费端也必须使用 Best Effort。
