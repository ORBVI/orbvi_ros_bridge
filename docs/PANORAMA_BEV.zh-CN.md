# 全景与 BEV

[English](PANORAMA_BEV.md) · [返回 README](../README.zh-CN.md)

## BEV/IPM

4V 和 4VL 默认通过 `streams` 中的 `bev` 开启板端优先的四路鱼眼 360° BEV/IPM。
`publish_ipm:=true` 开启的是同一路输出，只在覆盖 `streams` 且去掉 `bev` 时才需要：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch \
  streams:=raw,imu,vio publish_ipm:=true
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4v.launch.py \
  streams:=raw,imu,vio publish_ipm:=true
```

Bridge 先订阅板端 `bev_display_stream`，Host 只把板端 JPEG 解码为 `1000x1000`
`bgr8` ROS Image，板端目标频率为 20 Hz。仅设备明确返回该流不支持时才启用
Host 回退：默认假设四个相机离地 1.0 m，覆盖车体周围 20 m × 20 m，以
0.02 m/px 生成 1000x1000 BGR，最高 5 Hz。超时、流被阻塞或运行时故障都不会
触发板外派生。
以上为默认设备模式；使用完整安装外参和机器人地面平面的投影请见
[机器人 TF 与 IPM](ROBOT_INTEGRATION.md)。

`ipm_*` 参数见 [Launch 参数](LAUNCH_ARGUMENTS.zh-CN.md)。

## 全景

4V 和 4VL 默认请求板端优先全景：

```bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4v.launch
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4v.launch.py
```

公开 topic 固定在 `/orbvi` 下。两个 launch 都先请求板端 `pano_display_stream` 和
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
