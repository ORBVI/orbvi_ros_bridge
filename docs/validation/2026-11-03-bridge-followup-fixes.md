# 2026-11-03 bridge !72 修复与 rebase 验收

目标 MR：!72，源分支 `feat/localization-registered-cloud`。
以 `865f716210d839527f8be6c6441579e5b4c7ace8`（当时最新 main）为基础 rebase，
保留 !74 的 localization 命名、main 的 4V / 4VL 入口和拆分文档。
功能、命名迁移、后续修复保留为三个线性提交，没有增加 main 合并提交。

## 修复范围

- 雷达 IMU 单位参数缺失、无效或不支持时，仅禁用 `lidar_imu` 并告警；
  启动与转换共用单位和有限换算系数校验，正确缩放加速度及协方差。
- 可选 registered cloud 依据能力清单决定是否请求；旧固件没有清单则尝试订阅。
  主数据流先订阅，可选云最后订阅。单路失败不再导致所有输出退出；全部失败时返回非零。
- 直接订阅流增加异步首帧超时诊断，输出 SDK 阻塞原因，首帧恢复后报告恢复。
- 矫正压缩图兼容 V2 双 atlas、UVC/Bulk 单 atlas 和旧 JPEG bundle；
  校验 topic/frame_id、空 JPEG，未知格式不再静默忽略。
- ROS2 同 topic 串行发布，入队仅唤醒一个 worker；保留源时间戳。
- RViz 默认 visual 配置显示设备系深度点云；mid360 配置保留 1000 秒累计 RGB 云
  和独立 Live 开关，支持 fixed_frame 覆盖。
- 公开接口统一为 `/orbvi/localization/{odometry,path,imu_prediction}`。
  旧参数仅在新参数未设置时读取并告警；旧 topic 需消费者迁移或 remap。

## 编译与自动检查

以下均在 rebase 完成、launch 合约及 RViz 固定坐标系修正后执行，结果通过：

| 环境 | 编译 | CTest 入口 |
| --- | --- | --- |
| ROS1 Noetic / x86_64 Docker | 通过 | 3/3 |
| ROS2 Humble / x86_64 Docker | 通过 | 4/4 |
| ROS2 Foxy / x86_64 Docker | 通过 | 4/4 |

Foxy 检查覆盖 CI 所用 ROS 版本；本机镜像与云端 CI 镜像不相同，不能替代云端流水线结果。
测试覆盖 SI 单位与协方差、源纳秒时间保留、并发发布顺序和队列溢出、atlas 裁切与
缓冲区所有权、异常输入、可选流判定、launch 默认值。launch 合约 10 个方法通过。
ROS1 XmlLoader 实际解析 4V、4VL 和两种 RViz profile，以及 fixed_frame 覆盖均通过。

## 故障注入

真实 Network SDK 数据加控制面代理；每个案例在独立 ROS domain 下运行，窗口约 18 秒。

| 注入 | 结果 |
| --- | --- |
| 雷达 IMU 单位端点 HTTP 503 | 节点存活，雷达 IMU 0 条，主 IMU 3492 条、RGB 云 176 帧 |
| 能力清单不支持 registered cloud | 不发送该流订阅请求，主 IMU 3469 条，节点存活 |
| registered cloud 订阅 HTTP 404 | 云 0 帧，主 IMU 3470 条，节点存活 |
| 全部数据流订阅失败 | 节点退出，状态码 1 |
| 云订阅成功但数据连接无首帧 | 出现指定流超时告警，主 IMU 3469 条，节点存活 |

## 实机全流核对

Network SDK、ROS2 Humble，独立 domain 190、临时测试节点，默认 8 个发布 worker。
启用全部流、`publish_lidar_pcl:=true`、`publish_ipm:=true`，并行运行全输出检查与
IMU 专项检查。未停止用户原有 bridge，未修改或重启板端服务。
预热后连续统计 60 秒，51 项预期输出均收到有效样本，验证错误集合为空。

| 输出 | 接收频率 |
| --- | ---: |
| 四路原始图与压缩图 | 每路 20.00 Hz |
| 八路矫正图 | 每路 19.98 Hz |
| 八路矫正压缩图 | 每路 20.00 Hz |
| 四方向视差、深度与深度可视化 | 每路 10.00 Hz |
| 全景与 BEV | 各 20.00 Hz |
| 深度全景及其可视化、深度点云 | 各 10.00 Hz |
| 雷达 CustomMsg 与 PointCloud2 | 各 10.00 Hz |
| `/orbvi/cloud_registered` | 9.60 Hz |
| `/orbvi/localization/odometry` | 9.87 Hz |
| `/orbvi/localization/path` | 6.18 Hz |
| `/tf` | 9.60 Hz |
| `/tf_static` | 启动快照，7 条边，无周期频率 |

轨迹参数的 10 Hz 是发布上限。RGB 云未稳定达到每秒 10 帧，最大源时间间隔约 400 ms；
优化里程计最大间隔约 200 ms。仅凭 ROS 接收统计不能把这些间隔归因于板端算力。
RGB 样本含 234 点、216 种颜色，frame 为 `camera_init`；深度云样本含 17808 点。
八路 JPEG 均可解码为 320×256。以上周期输出未观察到时间戳倒序或重复。

IMU 专项结果（与全流统计为略有偏移的独立窗口）：

| 输出 | 帧数 | 频率 | 倒序 / 重复 | 加速度模长均值 |
| --- | ---: | ---: | --- | ---: |
| `/orbvi/imu` | 11870 | 197.83 Hz | 0 / 0 | 9.624 m/s² |
| `/orbvi/lidar/imu` | 11999 | 199.98 Hz | 0 / 0 | 9.765 m/s² |
| `/orbvi/localization/imu_prediction` | 12000 | 200.00 Hz | 0 / 0 | — |
| `/orbvi/localization/odometry` | 592 | 9.87 Hz | 0 / 0 | — |

专项检查的两路 IMU、位姿、速度及所检查的协方差无非有限值。
11999 个雷达 IMU 时间戳在预测时间戳集合中整数纳秒完全匹配。
本轮没有出现发布队列溢出或 JPEG 提取失败告警。
测试期间有一条优化 TF 样本因 `tracking_state=DEGRADED`、`tracking_reason=recovering`
被既有 TF gate 拒绝并告警；里程计消息仍发布，因此 TF 频率可以低于里程计频率。

## 验证边界与证据

本次仅修复 bridge，不解决板端时钟映射回跳、稠密点云生成或定位算法吞吐。
有限窗口未观察到倒序不代表板端未来时钟跳变已修复。
没有做 USB 硬件测试、主动校时跳变测试、ARM64 编译，也没有做 1000 秒 RViz 累积压力测试；
RViz 本轮验证配置与入口，未重新进行 GUI 留存时长验收。

本机证据目录：`Log/diagnostics/bridge-mr72-followup-fixes-20261103/`。
编译日志为 `build-{ros1,ros2,foxy}-final.log`，故障结果为各案例同名 JSON/log；
实机结果为 `rectified-fix-full-stream-audit.json`、`imu-focused.json`、
`full-flow-node.log`、`full-flow-result.json`、`ros1-launch-parse-final.log`。
