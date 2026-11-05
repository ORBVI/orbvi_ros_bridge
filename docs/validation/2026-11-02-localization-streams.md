# 2026-11-02 定位、点云、IMU 与矫正压缩图桥接验收

分支：`feat/localization-registered-cloud`，父提交 `9c274fc`。
环境：真实 4VL 的 Network SDK、Ubuntu 22.04 x86_64、ROS2 Humble；
板端使用 `db44d767` 基线加 `feat/livo-imu-prediction` 的 runtime 补丁
（提交 `ec1e7282`）。

## 最终行为

- 优化里程计、轨迹分别为 `/orbvi/localization/odometry`、`/orbvi/localization/path`。
- 新版 `RECTIFIED_FISHEYE_ATLAS_BUNDLE_V2` 通过公开 SDK 系数域无损裁切，转为
  八路 `/orbvi/rectified/<方向>/<左右>/image/compressed`，保留逐路源时间与 optical frame；
  兼容旧版独立 JPEG bundle，关闭拆分时不将拼接图冒充单路图像。
- SDK `registered_cloud_stream` 转为 `/orbvi/cloud_registered`，保留世界系、XYZI、真实 RGB。
- 两套 RViz 默认显示 1000 秒累计上色点云，保留独立实时单帧开关和四相机显示。
- 雷达 IMU 从公开 SDK 参数读取单位换算系数，加速度乘系数、协方差乘系数平方，
  保留不可用协方差标记；主 IMU 保持 SI 数值，时间戳和角速度均保留。
- ROS2 同一话题的发布任务串行执行，不同话题并行；默认仍是 8 个线程，
  全局待执行任务最多 64 个，满时丢弃最旧待执行任务并告警。停止时清空待执行任务并等待执行中任务结束。

## 乱序定位与自动测试

历史提交 `33c6877`（2026-06-17，`fix: enable ROS2 SHM publishing defaults`）
引入共享 FIFO 加 8 个发布线程，并把两路 IMU 纳入该队列。FIFO 出队之后解锁执行，
相邻同话题消息可以被不同线程反序发布。修复保留源时间，不通过改时间戳掩盖乱序。

修复前同板端对照：SDK 直订 35 秒，两路 IMU 均无倒序；默认 8 线程 ROS 订阅
30 秒分别出现 52／36 次倒序；仅改 1 个线程后两路均为 0。

Docker `local/orbvi-noetic-x86:20260923` 编译 ROS1，通过全部 3 个 CTest 入口
（32 项几何测试、44 项 SDK／转换／调度／JPEG 测试及 launch 合约）。
Docker `local/orbvi-humble-tf-test:20261015` 编译 ROS2，通过全部 4 个 CTest 入口
（32 项几何测试、46 项 SDK／转换／调度／JPEG 测试及两个 launch 检查）。

新增并发测试阻塞首条 IMU，确认同话题后续任务不能抢先，图像任务仍能执行；
另验证队列满时丢弃、保留任务的顺序、停止和重启。两套转换测试验证 SI 转换、
协方差／缺失标记、未知参数拒绝、主 IMU 不重复换算及纳秒源时间保留。
新增三项 JPEG 桥接测试：八路真实 JPEG 的内容、身份、独立时间戳和缓冲区所有权；
关闭拆分；元数据矛盾、越界、缺时间戳及第二张 JPEG 损坏时拒绝整帧。
灰度纹理的裁切 JPEG 解码后与原拼接图对应 ROI 逐像素完全一致。
`imgcodecs` 只用于测试生成和检查 JPEG，生产桥接通过 SDK 提取。

## 全流实机结果

安装新 bridge，清理板端定位重启前的旧 RViz 累计，重新启动全流和 RViz。
检查进程、RViz 使用与 bridge 相同的 `fastdds_shm.xml`；CLI 使用
`--no-daemon`，避免旧 UDP discovery daemon 造成“Node not found”。
实际读取 `publish_worker_threads` 为 8。

补齐压缩图后的最终构建，专项 ROS 订阅 60 秒：

| 输出 | 帧数 | 接收频率 | 时间戳倒序／重复 |
| --- | ---: | ---: | ---: |
| `/orbvi/imu` | 11858 | 197.63 Hz | 0／0 |
| `/orbvi/lidar/imu` | 11993 | 199.88 Hz | 0／0 |
| `/orbvi/vio/imu_prediction` | 11997 | 199.95 Hz | 0／0 |
| `/orbvi/localization/odometry` | 599 | 9.98 Hz | 0／0 |

雷达 IMU 加速度模长均值 9.759 m/s²，主 IMU 为 9.600 m/s²；
两路 IMU 及两类位姿的数值和协方差均为有限值。
11991 个预测时间戳与接收到的雷达 IMU 时间戳整数纳秒完全一致；
独立接收窗口／传输缺帧导致两集合并不完全相等，不伪造采样补齐。

八路矫正压缩图各收到 1197–1198 帧／60 秒，约 19.95–19.97 Hz，均可解码为
320×256 JPEG，零倒序、零重复；每路进行 62 次与同时间戳非压缩图的像素对照。
平均绝对像素差为 0.97–1.06／255，SDK 与 OpenCV 的 JPEG 解码路径不同；
没有出现左右目或方位错配，检查错误列表为空。

并行全 topic 检查中，51 项预期 ROS 输出均收到有效样本（静态 TF 为启动快照）。
四相机非压缩图各约 19.45 Hz，校正图约 20 Hz，全景／BEV 约 19.8／20 Hz，
雷达约 10 Hz、上色点云约 9.95 Hz、深度点云约 9.80 Hz。
轨迹约 6.15 Hz（配置 10 Hz 是上限），TF 约 9.95 Hz；静态 TF 收到 7 条边。
收到消息的 topic 均未发现倒序；RGB 样本为 236 点、212 种采样颜色。
RViz 截图确认四相机、累计上色点云、定位轨迹及 TF 正常显示。
全流期间没有 JPEG 提取失败或发布队列溢出告警；板端服务 active、NRestarts 为 0。
板端四个部署产物的 SHA256 与测试 stage 全部一致。

SDK 的 `frame_sync_stream` 可订阅，bridge 尚无对应的公开 ROS 输出契约，
不能将其列为已承诺而缺失的 ROS 话题。
本次没有恢复稠密上色；上色点云仍使用板端定位降采样后的点。
累计保留 1000 秒的配置已检查，本轮未做完整 1000 秒留存时长试验。

现场证据在工作区 `Log/diagnostics/bridge-localization-20261102/`：
`ros1/rectified-jpeg-build-test.log`、`ros2/rectified-jpeg-build-test.log`、
`desktop-rectified-jpeg-install.log`、`rectified-jpeg-audit.json`、
`rectified-fix-full-stream-audit.json`、`rectified-fix-imu.json`、
`rectified-fix-full-streams-bridge.log`、`rectified-fix-board-health.log`。
先前 IMU 线程数和 RViz 截图分别为 `imu-fix-worker-parameter.log`、`imu-fix-rviz.png`。
设备地址及现场配置保留在该目录，不进入源码提交。
