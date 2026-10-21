# 安装 TF 单一来源与启动 odom 修复验收（2026-10-15）

本轮在正式 `orbvi_ros_bridge` 仓库、基于 `257ccb5` 实现客户端修复。
完整提交号见交付包 `manifest.json`。链路保持板端 native → SDK → 用户主机
ROS bridge；未修改/重启板端服务，没有引入板端 ROS 依赖。
本记录取代上一轮复核中“URDF 安装数值仍有两份、尚无启动 odom”的待修项。

## 最终行为

- URDF 模式禁止 bridge YAML 中出现 xyz/rpy，仅保留安装 parent/child 与可选
  机器人地面平面。机器人动态 TF 查询实际 `T_base_mid360_imu`，IPM 查询实际
  `T_base_camera`，两者共用同一 tf2 buffer 和静态路径审计；不回退另一份安装值。
- bridge 模式要求显式 xyz/rpy，并将自己的安装边送入相同解析流程。
  `publish_sensor_tf=false` 时只在本地 buffer 使用该配置，不向网络发布静态边。
- 新增条件 xacro 示例；由集成 launch 的同一个 publish_mounting_tf 值控制
  URDF joint 和 bridge。持续按 callerid/GID 审计静态路径。重复发布、parent/数值
  变化、动态安装和环路会暂停依赖输出，保留原始 topic/Odometry。
  冲突锁定，修正后重启；不承诺能阻止独立外部节点或清除消费者已缓存的旧 TF。
- 缺安装 TF 时等待，延迟到达自动恢复。独立相机分支可在缺 IMU 锚点时支持 IPM；
  IMU 连接稍后补齐时可恢复机器人位姿。非法静态分支不替换成单位变换。
- 新增默认关闭的 publish_odom_tf / odom_frame。首次有效机器人位姿定义一次
  `odom → camera_init`，启动 footprint 竖直投影为原点、机器人水平航向为 +X，
  保持 native Z-up 和机器人实际倾角。需显式机器人系 ground_plane。
  拒绝已有世界 parent 或 odom 名称落入机器人静态子树，避免环路。
  已识别的算法重启会暂停此模式，要求重启 bridge，避免旧锚点跨代复用。
- 地面平面继续表达在 base/安装 parent 下。相机 ID 仅匹配输入；方位、有效视野、
  投影权重由完整相机 TF 和光学射线确定，没有按 ID 固定机器人方位。
- 单帧 IPM 沿用图像采样时间。未新增世界系 BEV 累积，也没有为累积实现位姿缓存。
  `mid360_imu → aft_mapped` 单位别名仍保留以兼容原 Odometry；无旧动态边。

## 环境与版本

| 项目 | 版本/状态 |
| --- | --- |
| 主机 | Linux x86_64，Docker，SDK 地址 192.168.3.100:18088 |
| ROS1 构建镜像 | `local/orbvi-noetic-x86:20260923` |
| ROS2 构建镜像 | `local/orbvi-humble-tf-test:20261015` |
| SDK SOURCE_FINGERPRINT | `94ccc39ee2e61406a22a6e1e5145a1bc3c7ba48364f23afbf23b12a65808653d` |
| 板端 release | `release-4vl-20261015-g1bae9e43-localization-r9` |
| 算法服务 | `orbvi_omni_livo.service` active，PID 15416（检查时） |
| native status | `world_frame=gravity-aligned-z-up-v1` |
| 相机标定 hash | `93390712682fdbf6d0a7c28058b8bae254f4583f7c4d31071d8d32346f5d87bd` |
| 雷达标定 hash | `d0983a6325fe11b39ac01216447626fa212031ed11aa6d52e903b6beae80a68f` |

板端只读 status 原始记录为 `board/ownership-runtime.txt`。
启动 odom 只校正原点/yaw，不再次根据安装姿态做重力调平；旧 SDK 无自动世界
语义探测，旧 native 或关闭重力对齐的部署须先确认世界系语义。

## 构建、单测与实时收发

两版构建、32 项几何/IPM/session 单测、Host SDK 回归与 7 项 launch 契约测试通过。
测试工具最终摘要：Noetic `134 tests, 0 errors, 0 failures`（catkin 汇总计数），
Humble `72 tests, 0 errors, 0 failures`；launch 独立运行均为 7/7。
新增用例包含 URDF 拒绝第二套数值、真实外部 TF 替代占位配置、缺边等待/恢复、
重复发布者/变化/动态/环路、TF 关闭时本地几何、启动地面偏移/yaw/实际倾角，
以及算法重启和 odom 名称冲突。机器人 IPM 的 TF 路径结果与原完整外参路径的
合成图逐字节一致；仍覆盖正装、倒装和旋转安装。

每版 9 组直接节点收发（各约 12 秒），再加 1 组正式 launch + 实际
robot_state_publisher（约 14 秒），**20 组均通过**。
表中为动态 TF / 原始 IMU Odometry / 机器人 IPM 收到条数：

| 用例 | ROS1 Noetic | ROS2 Humble |
| --- | --- | --- |
| bridge 正装 | 111 / 116 / 225 | 111 / 117 / 228 |
| bridge 倒装配置 | 111 / 115 / 225 | 111 / 117 / 228 |
| URDF 外部静态边 | 114 / 116 / 231 | 115 / 117 / 235 |
| URDF 延迟 4 秒 | 77 / 115 / 158 | 73 / 116 / 158 |
| 启动前已有重复安装发布者 | 0 / 114 / 0 | 0 / 116 / 0 |
| 启动后 3 秒注入重复发布者 | 24 / 117 / 48 | 21 / 117 / 48 |
| 启动 odom（倒装配置） | 114 / 117 / 227 | 113 / 118 / 228 |
| TF 全关，IPM 保持 | 0 / 116 / 234 | 0 / 116 / 235 |
| 默认设备模式回归 | 103 / 112 / — | 106 / 114 / — |
| 正式 launch + robot_state_publisher + odom | 125 / 131 / 269 | 128 / 133 / 269 |

- 所有已配对动态 TF 均与原始 Odometry 采样时间相同，独立矩阵组合误差 ≤1e-9。
- 相同输入的两版静态传感器/安装边数值一致，最大差为 0（启动 odom 依赖不同
  首帧位姿，不作为两版固定矩阵比较对象）。动态时间戳严格递增。
- 完整机器人树为 8 条静态边；开启启动 odom 为 9 条。晚加入订阅者收到全部
  静态边。四路相机、雷达、IMU、设备 frame、原 IMU 别名均可查询。
- 延迟注入前机器人动态 TF/IPM 均为 0；注入后恢复。运行中冲突发生后，观察
  1.5 秒处理窗口，再验证动态 TF/IPM 计数不再增加。原始图像、雷达 IMU、
  ROS_BODY 深度点云及 Odometry 在冲突期间继续收到。
- 两版实际 xacro 展开：URDF 模式恰有 1 个安装 joint、2 个 link；bridge 模式
  无安装 joint、仅保留机器人根 link。实际 robot_state_publisher 收发用例通过。
- 正式 ROS2 launch 的探针/RSP 与 bridge 使用相同 Fast DDS SHM profile、RMW，
  Docker `/dev/shm=2g`；直接节点矩阵使用默认本地 DDS 配置。

安装产物迁移到 `/opt/orbvi_bridge`、不挂载源码后，ROS1/ROS2 的库加载和
导出工具均通过。bridge-owned 模式各导出 8 条，与实测广播逐值一致，最大差 0；
URDF 模式各导出 7 条内部边，未冒充读取外部安装值。

## 复现与原始证据

构建容器挂载 `REPO:/source:ro`、`ROOT/noetic:/build` 或 `ROOT/humble:/build`，
工作目录 `/build/ws`。REPO 是正式 bridge 仓库，ROOT 为
`/home/xingmaker/huanshi/build/tf-bridge-20261015`。

```bash
# Noetic
source /opt/ros/noetic/setup.bash
catkin_make -j4
catkin_make -j4 run_tests_orbvi_ros_bridge
catkin_test_results
python3 /source/orbvi_ros_bridge/test/test_launch_contracts.py
catkin_make -j4 install
# Humble
source /opt/ros/humble/setup.bash
colcon build --packages-select orbvi_ros_bridge --parallel-workers 4
colcon test --packages-select orbvi_ros_bridge
colcon test-result --verbose
python3 /source/orbvi_ros_bridge/test/test_launch_contracts.py
```

收发命令及参数完整保留在交付包的 `evidence/ownership-matrix.sh` 和
`ownership-rsp-launch.sh`；由相应 ROS 镜像运行，版本参数为 1 或 2。
矩阵调用仓库 `orbvi_ros_bridge/test/probe_tf.py`，独立 ROS1 master 11461、
ROS2 domain 164；正式 launch 用 master 11462 / domain 165。
`--mounting-reference` 只用于测试端独立校验/模拟 URDF，生产 bridge 不读取它。
正式 launch 测试使用 `--external-mount-publisher`，由真实 RSP 发布安装边。

原始结果位于 ROOT 下的 `{noetic,humble}/ownership-*.json`、对应
`.bridge.log` / `.probe.log`、`ownership-build.log` 与构建 XML；两版汇总为
`ownership-cross-version.json`。交付包保留上述证据、展开的 URDF、源码、配置、
导出示例及校验清单。导出仅支持内部标定/bridge-owned 安装边，不读取外部 URDF
或动态/启动 odom。

## 范围与迁移

本轮验证数学组合、发布、所有权、功能恢复；示例安装值不代表真实机器人测量。
未倒装板卡重新上电，未验证物理标定/投影精度；未重跑十分钟运动或完整资源基线。
不据短时消息收发宣称长期 RSS/CPU 或定位质量达标。没有修改板端初始化策略。
世界系多帧 BEV 累积若后续实现，必须按图像采样时间查位姿并处理等待/插值。

URDF 用户须改用 frame-only mounting 配置并提供实际静态安装边；已有 numerical
YAML 仅用于显式 bridge-owned 模式。升级/回滚需成套恢复 bridge、launch 和安装
配置。默认 device 模式保持兼容；回滚包保留在旧版交付目录，未向客户远程发布。
