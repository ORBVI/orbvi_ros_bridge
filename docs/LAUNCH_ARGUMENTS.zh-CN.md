# Launch 参数

[English](LAUNCH_ARGUMENTS.md) · [返回 README](../README.zh-CN.md)

对外接口固定为 4V 和 4VL 两套 launch 配置，见 [启动 Bridge](../README.zh-CN.md#启动-bridge)。
两套 launch 默认开启对应型号的全部能力，调用方可以覆盖 `streams`。公共参数如下。

## 公共参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `192.168.3.100` | ORBVI 设备 IP 或 hostname |
| `streams` | 4V：`raw,rectified,pano,bev,imu,disparity,depth,localization`；4VL：`raw,rectified,pano,bev,imu,lidar,lidar_imu,disparity,depth,localization,registered_cloud` | 逗号分隔的 stream token；`pano`、`bev` 表示板端优先显示流 |
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
| `publish_localization_path` | `true` | 将优化后的 `/orbvi/localization/odometry` 累计并发布到 `/orbvi/localization/path` |
| `localization_path_max_poses` | `2000` | Path 中保留的最大 Pose 数；小于等于 0 时按 1 处理 |
| `localization_path_publish_rate_hz` | `10.0` | Path 最大发布频率；设为 0 时随每条优化 odometry 发布 |

Topic 名称固定在 `/orbvi` 下。BEV/IPM 与全景行为见 [全景与 BEV](PANORAMA_BEV.zh-CN.md)。

## 定位接口迁移与数据流兼容

位姿、轨迹和预测统一使用 `/orbvi/localization/{odometry,path,imu_prediction}`。
旧消费者需修改订阅或使用 ROS remapping。launch 参数使用 `publish_localization_tf`、
`publish_localization_path`、`localization_path_max_poses` 和 `localization_path_publish_rate_hz`。
直接启动节点时，新参数未设置才读取旧的 `publish_vio_*` / `vio_path_*` 参数并告警；
launch 入口只接受新参数。`localization`、`vio` 和旧的 `vio_pose_stream` token
均展开为 `optimized_pose_stream` 与 `predicted_pose_stream` 两路订阅，不再订阅
已废弃的混合流；也可单独指定任一新流名。优化位姿发布里程计、轨迹和动态 TF，
预测位姿只发布 `/orbvi/localization/imu_prediction`。

雷达 IMU 根据 SDK 单位参数将加速度转换为 m/s²，协方差乘换算系数的平方，
保留 `-1` 不可用标记。启动与转换共用单位和有限 scale 校验；参数缺失或无效时，
仅跳过 `lidar_imu` 并告警，不猜测单位。

4VL 默认请求可选 `registered_cloud`；能力清单明确不支持时跳过，旧设备没有清单则
尝试订阅。主要流先订阅，可选注册点云最后订阅。单路订阅失败保留其他流；
没有任何订阅成功时才启动失败。异步订阅超过 `first_frame_timeout_ms` 未收到首帧时，
按流告警并附 SDK 原因，其他流继续；修复设备能力或配置后需重启 bridge 重新订阅。

矫正压缩图兼容旧 JPEG bundle、V2 双 atlas 以及 UVC/Bulk 单 atlas，使用 SDK 无损
裁切获得八路 JPEG。无效的 topic/frame_id、空图像及未知格式会拒绝输出并提供诊断。

ROS2 发布队列按 topic 串行，不同 topic 可并行；入队唤醒一个 worker。
队列满时丢弃最旧的待执行任务并节流告警。源采样时间戳保持不变，这不能修复板端时钟回退。

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

`/orbvi/localization/path` 不使用 `/orbvi/localization/imu_prediction`。每个 Pose 沿用优化 odometry
的坐标系、时间戳和位姿；坐标系变化或时间戳回退时自动清空旧轨迹。默认最多保留
2000 个 Pose，内存和单条消息大小均为 `O(localization_path_max_poses)`，整条 Path 默认最多
发布 10 Hz。高频或低带宽场景可调小点数或频率上限，或设置
`publish_localization_path:=false` 关闭。

4VL 的板端 Omni-LIVO 使用同一位姿流，bridge 通过
`localization_source=omni_livo` 和 `pose_kind=OPTIMIZED` 识别优化位姿，
并发布到 `/orbvi/localization/odometry` 和 `/orbvi/localization/path`。

ROS1/ROS2 bridge 均支持 Omni-LIVO 动态 TF 与标定静态 TF：
默认设备模式下，`camera_init → aft_mapped` 连接雷达、雷达 IMU、四路原始相机及 SDK ROS_BODY 深度点云。
静态 TF 在标定加载后立即发布，不等待位姿。`publish_localization_tf` 默认开启；
`publish_sensor_tf` 默认继承它，也可单独开启。新增 `orbvi_export_extrinsics`
导出带标定 hash 的外参 JSON。原始相机 frame 统一为 `orbvi/camera/<SDK camera ID>`，
雷达 IMU frame 修正为 `mid360_imu`；传感器话题名称保持原契约，雷达 IMU 加速度转换为 ROS 要求的 m/s²。
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
