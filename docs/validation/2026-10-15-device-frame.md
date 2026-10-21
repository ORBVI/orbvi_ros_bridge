# 设备参考系命名迁移验证（2026-10-15）

本次在 `e01162f` 的 SDK ROS bridge 上将默认设备参考系由 `base_link` 改为
`orbvi_link`，同步修改 ROS_BODY 深度点云的 frame。原点、坐标轴、标定数值、
原始相机光学坐标系名称及 `camera_init → aft_mapped` 动态位姿链不变。
板端 runtime、SDK 二进制和服务均未改动。

`depth_pointcloud_frame_id:=base_link` 可恢复原名称；导出工具相应使用
`--device-frame base_link`。六个用户 launch（ROS1/ROS2 各三套）透传参数。
设备相对机器人底盘的安装外参和 IPM 地面平面不在本次命名迁移中生成。

## 验证结果

- Noetic Docker 构建通过：34 项 SDK/消息转换测试、10 项几何/位姿门控测试全部通过；
  7 项 launch 契约测试通过。catkin XML 汇总 88 为嵌套计数，不代表 88 个独立 C++ 测试。
- Humble Docker 构建通过：colcon 汇总 49 项，0 失败、0 错误、0 跳过；
  另执行 7 项 launch 契约测试通过。
- 新增回归用例确认新旧 frame 名下全部静态变换的平移和四元数逐值一致，
  默认树不再占用 `base_link`。
- 同一本地 4VL 板卡分别连接 Noetic/Humble bridge。默认 `orbvi_link` 和旧名
  `base_link` 各采集 12 秒：7 条静态边、晚加入订阅、四路相机/雷达/IMU 的
  tf2 查询均通过；实际 ROS_BODY 点云 header 分别与两个配置一致。
- 新旧命名导出与两个 ROS 版本广播结果的最大数值差均为 0。与原始标定矩阵
  独立组合结果保持原有舍入差（最大约 `5.864443e-7`，阈值 `2e-6`）。
- ROS1 launch 空字符串参数解析通过。Humble 实际 launch 启动并加载 7 条静态边；
  Docker 需 `--shm-size=1g` 以容纳已有 Fast DDS 的 512 MiB SHM profile。
  首次使用默认 Docker SHM 的启动失败日志保留，未为该环境问题更改产品配置。

## 动态位姿复测边界

本轮最初按 full 模式采集 20 秒，两个 ROS 版本均收到静态 TF、图像、雷达 IMU 和
深度点云，但没有收到 Odometry，因此 full 模式报告如实失败。只读检查显示
`orbvi_omni_livo.service` 为 `ActiveState=failed`、`MainPID=0`。本轮没有重启或
修复板端服务，也不将本轮静态结果称为动态 TF 实板通过。

随后明确以 `static-only` 模式、不订阅 vio 流验证命名迁移。动态路径本次未修改，
原实现的动态实板证据见 [原 TF 验收](2026-10-15-omni-livo-tf.md)。
本轮不开展物理标定精度验收。

## 命令与证据

证据根目录为工作区外的 `build/tf-bridge-20261015/`；设备数据不提交 Git。

```bash
# 对应 ROS Docker 环境中执行
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release
catkin_make -j4 run_tests_orbvi_ros_bridge
colcon build --packages-select orbvi_ros_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select orbvi_ros_bridge
colcon test-result --verbose
python3 /source/orbvi_ros_bridge/test/test_launch_contracts.py

python3 /source/orbvi_ros_bridge/test/probe_tf.py --ros ROS_VERSION \
  --mode static-only --host DEVICE_IP --bridge-executable BRIDGE_BINARY \
  --duration 12 --include-depth --output REPORT.json
# 兼容配置追加 --device-frame base_link
orbvi_export_extrinsics --host DEVICE_IP --output NEW_FILE.json
# 兼容配置追加 --device-frame base_link
```

- `noetic/device-frame-build.log`、`humble/device-frame-build.log`：构建和测试。
- 各 ROS 目录 `device-frame-static.json`、`device-frame-legacy.json`：通过的实际收发报告。
- 各 ROS 目录 `device-frame-full.json`：无 Odometry 的初始 full 模式报告。
- 各 ROS 目录 `device-frame-extrinsics.json`、`device-frame-legacy-extrinsics.json`：导出。
- `board/device-frame-comparison.json`、`board/device-frame-legacy-comparison.json`：独立组合与跨版本广播对比。
- `noetic/device-frame-launch-params.yaml`：ROS1 launch 解析结果。
- `humble/device-frame-launch-shm.log`：Humble 实际 launch 成功启动日志。
