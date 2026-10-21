# Omni-LIVO bridge TF 本地验收记录（2026-10-15）

本次改造位于正式 `orbvi_ros_bridge` 仓库，板端未加入 ROS 依赖、未更新 SDK/位姿协议或设备服务。整合了工作区已有 ROS2 TF、Reliable QoS、launch 和 RViz 改动。源版本以本报告所属 Git 提交及产物 manifest 为准；对照版本为 `76916d4`。

**部署说明：** 已与用户确认：板端 Omni-LIVO/native → SDK → 用户主机上的 `orbvi_ros_bridge` → ROS 消息与 TF。本次入口始终为独立 SDK bridge，板内另行运行的 `orbvi_ros_adapter_node` 不在改造范围，未改动其代码或服务。本报告 x86 Docker 结果覆盖用户主机上的 ROS1 Noetic/ROS2 Humble bridge；不把它描述为板端 native/adapter 性能。

## 环境与原始证据

- ROS1：Noetic / Ubuntu 20.04 / x86_64，Docker `local/orbvi-noetic-x86:20260923`。
- ROS2：Humble / Ubuntu 22.04 / x86_64，基于 `ros:humble-ros-base-jammy` 的本地构建环境。
- SDK：随仓库发布的 0.1.0 动态库，SOURCE_FINGERPRINT `94ccc39ee2e61406a22a6e1e5145a1bc3c7ba48364f23afbf23b12a65808653d`。
- 板端：4VL，`orbvi_omni_livo.service`，原生进程 `omni_livo_native_node`；设备版本 `release-4vl-20261014-g1bae9e43-y-jac-passive-rawbuffer-stability-20261014-r2-trace-r3-correlation-r4`，构建 ID `1bae9e43f35e61e35c5ec33d5d6e0934ae326ca5`。
- 本地证据根目录：`/home/xingmaker/huanshi/build/tf-bridge-20261015/`，包含 `noetic/`、`humble/`、`board/`。实际设备标定、图像和录包只保存在该目录，未加入源码提交。
- 本轮运动前有效配置为 `/opt/orbvi/current/share/omni_livo/config/mid360_native.yaml`，SHA256 `da4da5ac98acc41ce39c4b8f5eb6eca353d5838a0479601f0c844d76fb7cca72`。早期准备阶段还观察到 `/run/orbvi/density-test/filter-0p2.yaml`，不将两个配置下的结果混作同一受控基线。

| 标定 | SHA256 / SDK hash |
| --- | --- |
| 相机 rig / camera_config.yaml | `93390712682fdbf6d0a7c28058b8bae254f4583f7c4d31071d8d32346f5d87bd` |
| 雷达 SDK bundle | `d0983a6325fe11b39ac01216447626fa212031ed11aa6d52e903b6beae80a68f` |
| 前相机—雷达锚点文件 | `a698f99d614179c9d71fa91227e0f46c3155bddc61d54bb554b7700c6e7a94a2` |
| 雷达 IMU 文件 | `21a99ab10564c4100b6cf5aff20f9ad846d57e3852731f908d3f09ccb37c79f5` |

IMU 外参来源为板端标注的 `manufacturer_nominal`，本次核对其使用路径与数值，未将其描述为现场重新标定。

## 已完成验证

| 项目 | 结果与证据 |
| --- | --- |
| ROS1 构建/单测 | 34 项消息转换/SDK 链接测试 + 9 项共用几何/位姿门控测试通过，7 项 launch 测试通过；`noetic/build-release.log`。catkin 汇总会重复计入嵌套 testsuites，真实 C++ 用例数按 XML 根结点计为 43。 |
| ROS2 构建/单测 | colcon 汇总 48 项，0 错误、0 失败、0 跳过；`humble/build-release.log`。 |
| 开关矩阵 | 两版 full、sensors、disabled、static-only、dynamic-only 均通过；ROS1 `live-*.json`，ROS2 `live-full.json`、`final-*.json`。关闭 TF 后仍收到 Odometry/原始图像/雷达 IMU。 |
| 静态 TF | 7 条边，四路相机身份为 3/2/4/5，雷达 IMU 为 mid360_imu；无 vio 订阅也能发布；晚加入订阅者收到完整树。 |
| 动态 TF | 仅优化位姿，原采样时间戳；实板配对数据误差不超过 1e-9；最新及给定位姿采样时刻均可通过 tf2 查询。 |
| 连续静止 | ROS1 600 秒，5,997 对 Odometry/TF；ROS2 660 秒，6,561 对；均未发现配对差异。用户确认该阶段未移动设备，故只记作静止验证。 |
| 导出一致性 | `noetic/board-extrinsics.json` 为实板导出样例。独立 NumPy 组合核对通过，导出与 ROS1/ROS2 静态广播最大差 0；`board/geometry-comparison.json`。 |
| SDK 暂断 | 两版独立 Docker 实例分别断网 5 秒，约 5.3 秒数据缺口后恢复，恢复后 tf2 查询与位姿配对通过；各版本 `network-fault.json`、`network-recovery.json`。未断开主运动采集实例。 |
| 安装/重启 | 多次独立启动 bridge 均恢复静态树及动态位姿；ROS1 导出工具移除源码挂载、迁移安装前缀后仍可读取标定。 |

前锚点旋转矩阵由于原文件小数舍入，正交误差为 `1.186091e-6`；转换为归一化四元数后，独立原矩阵组合对比的最大差为 `5.864443e-7`。原矩阵核对使用单独记录的 2e-6 容差，不能与动态 TF/Odometry 的 1e-9 要求混淆。没有改写设备标定。

## 运动、资源及边界

运动采集于北京时间约 10:20:23 开始，用户确认本轮已做平移与转动；结果见下方追加的实测记录。录制全部 TF/Odometry，四路图像、雷达点云和雷达 IMU 每 topic 最多 1 Hz；这不是无损传感器录制。

早期录包暴露两个验证工具问题：ROS1 多回调并发写 rosbag，ROS2 在线读取正在写的 SQLite 导致锁竞争。已为录包写入加锁、增加信号收尾及错误摘要，后续正式运动轮重新录制。早期 bag 仅作为辅助证据，不计作完整运动通过依据；`noetic/final-samples.summary.json` 为修正后短程验证。

板端没有在当前位姿元数据提供 boot_id/generation。已验证门控在显式新代时允许重置并拒绝旧代重返；无代标记时只允许时间向前的序列重置，并输出诊断。无法安全识别的向后时钟重置需要重启 bridge，不能声称协议层已补齐这一能力。

测试准备/早期静止阶段观察到板端进程与启动配置由外部变化（PID 3082→4325→6292）。本次未发送任何板端启停或配置写入命令，不将这些变化解释为 bridge 故障注入结果。算法重启只做了门控单测与外部实际变化观察，未主动破坏板端运行。

几何单测和标定文件核对验证坐标组合；尚未进行带地面真值的运动精度评测或同步图像—雷达重投影精度评测。已有算法定位质量问题不归为本次 TF 修复成果。ARM64 Noetic 构建曾短暂启动，随后按明确的用户主机部署范围停止，未计作通过；未做客户环境发布或 ROS2 Foxy 本地实测。

本轮运动实测（`noetic/actual-motion.json`、`humble/actual-motion.json`）：

| 项目 | ROS1 | ROS2 |
| --- | --- | --- |
| 运行时间 | 660.42 秒 | 660.42 秒 |
| 配对 Odometry/动态 TF | 6,597 对 | 6,599 对 |
| 配对误差 | ≤1e-9 | ≤1e-9 |
| 源位姿频率 | 10.0001 Hz | 10.0001 Hz |
| 最大源时间间隔 | 0.105 秒 | 0.105 秒 |
| 最新/采样时刻查询、晚加入静态树 | 通过 | 通过 |

两版录包的 6,591 个共同源时间戳位姿逐值相同（最大差 0），证据为 `motion-cross-version.json`。

ROS2 录包位姿三轴范围为 `[0.6135, 0.6052, 0.3842]` 米，相对起点最大转角 `136.45°`，证实本轮包含实际运动。两版 recorder 摘要均无错误、无中断，约各 0.9 GB，含四路相机、雷达、雷达 IMU、TF、Odometry；传感器按 1 Hz 抽样，ROS2 recorder 的 TF 数量少于探针收到的数量，不能描述为无损录制。结束后板端 native 仍为 PID 6292、active，adapter PID 3269，均未因停止这些 SDK bridge 而停止。

同机、同板、相同 `vio,raw,lidar_imu` 输入的 90 秒历史 HEAD/当前版本 CPU 对比（去除前 10 秒热身）：ROS1 单核占用约 `120.60%→125.13%`，增量 `4.52` 个百分点；ROS2 使用两版均兼容的 Best Effort 传感器订阅，约 `133.43%→135.41%`，增量 `1.98` 个百分点。该数值是 x86 客户端进程，不是 ARM64 板端开销。旧版无法路由这块板的 Omni-LIVO 优化位姿，Odometry 为 0，不能计算有意义的“位姿频率下降比例”。

660 秒运动轮 RSS 范围：ROS1 约 57–79 MiB，ROS2 约 91–126 MiB；早期稳定窗口到末尾窗口的 RSS 中位数仍分别增加约 3.8 / 5.2 MiB，不据此宣称长期无增长。CPU 样本、内存样本保存在各 JSON 的 resource_samples；专用板端基线及更长时间内存平台观察仍待有需要时在目标用户主机完成。

## 复现命令

在对应 ROS Docker 环境内（source ROS 和 workspace 后）：

```bash
# ROS1
cd /build/ws
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release
catkin_make -j4 run_tests_orbvi_ros_bridge
python3 /source/orbvi_ros_bridge/test/test_launch_contracts.py
# ROS2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select orbvi_ros_bridge
colcon test-result --verbose
# 对真实设备仅作读取；ROS1 使用独立 master，ROS2 使用独立 domain
orbvi_export_extrinsics --host DEVICE_IP --control-port 18088 --output NEW_FILE.json
python3 /source/orbvi_ros_bridge/test/probe_tf.py --ros 2 --host DEVICE_IP \
  --bridge-executable /path/to/orbvi_ros_bridge_node --duration 660 \
  --mode full --include-lidar --output actual-motion.json
python3 /source/orbvi_ros_bridge/test/record_tf_samples.py --ros 2 \
  --duration 660 --output actual-motion
python3 /source/orbvi_ros_bridge/test/compare_extrinsics.py \
  --calibration calibration.json --export extrinsics.json \
  --ros-report ros1-full.json --ros-report ros2-full.json --output comparison.json
```

外参、frame 迁移、launch 开关和回滚定义见 [TF.md](../TF.md)。只需客户端升级即可使用本次功能。回滚恢复上一版 bridge 安装目录及配套 launch；不改动板端 runtime。产物在本地交付，不推送远程或发布给客户。
