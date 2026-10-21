# 机器人安装 TF 与地面 IPM 验证（2026-10-15）

本轮基于 `da34806`，修改正式 `orbvi_ros_bridge`，部署链路仍为板端 native →
SDK → 用户主机 ROS bridge。源提交由交付 manifest 记录。未修改板端服务、SDK
二进制或协议。验证计算、TF 发布与图像输出，不评测物理标定精度。

## 已确认的坐标语义

Omni-LIVO native 的 `PublishPose` 输出 `state_.pos_end/rot_end`，
`LivoPoseMsgbusPublisher::BuildMessage` 标注 `camera_init/aft_mapped`；状态是
MID360 IMU 系。雷达点转换还要乘雷达—IMU 外参，不能把状态原点当成雷达或设备中心。
SDK ROS_BODY 使用参考光学系到前/左/上的 `(x,y,z) → (z,-x,-y)` 轴变换，
原点沿用相机 rig，bridge 命名为 `orbvi_link`。原始相机为光学系。

机器人模式发布 `camera_init → base_link → orbvi_link → sensors`，动态位姿为
`T_world_imu * T_imu_device * inverse(T_base_device)`。内部标定与安装 YAML 分离；
安装边默认由 URDF 发布，可显式改由 bridge 发布。保留 `mid360_imu → aft_mapped`
静态别名，原 IMU Odometry/Path 不变。没有连接锚点时不发布机器人动态位姿，保留
独立有效的相机、雷达和 IMU 分支，不用单位外参补齐。

IPM 使用四路原图、SDK triple-sphere 内参、完整相机外参、安装外参和机器人地面
平面计算。输出顶部为机器人前方、左侧为机器人左方，frame 是 mounting parent。
样例正装/倒装 YAML 的位置和地面平面仅用于功能测试，不能当作用户安装实测值。

## 环境和证据

- Noetic / Ubuntu 20.04 / x86_64，`local/orbvi-noetic-x86:20260923`。
- Humble / Ubuntu 22.04 / x86_64，`local/orbvi-humble-tf-test:20261015`。
- SDK 0.1.0，SOURCE_FINGERPRINT：
  `94ccc39ee2e61406a22a6e1e5145a1bc3c7ba48364f23afbf23b12a65808653d`。
- 本地 4VL 板 SDK `192.168.3.100:18088`；设备版本及历史运行记录见
  [原 TF 验收](2026-10-15-omni-livo-tf.md)。本轮只读检查
  `orbvi_omni_livo.service` 为 `MainPID=0, ActiveState=failed, SubState=failed`。
  图像、深度、雷达 IMU 与标定读取正常，本轮未重启该服务。
- 相机 hash：`93390712682fdbf6d0a7c28058b8bae254f4583f7c4d31071d8d32346f5d87bd`。
- 雷达 hash：`d0983a6325fe11b39ac01216447626fa212031ed11aa6d52e903b6beae80a68f`。
- 原始证据根目录：`/home/xingmaker/huanshi/build/tf-bridge-20261015/`。
  下表路径均相对该目录。设备标定和真实图像数据不加入源码。

## 结果

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| ROS1 编译/测试 | 35 项 SDK/ROS 消息测试 + 22 项几何、机器人 TF/IPM 测试通过，另 7 项 launch 契约通过；catkin 汇总的 114 为嵌套重复计数 | `noetic/robot-final-build.log` |
| ROS2 编译/测试 | colcon 62 项，0 错误、0 失败、0 跳过；另 7 项 launch 契约通过 | `humble/robot-final-build.log` |
| 正装与倒装静态树 | 两版均为 8 条完整边，每个 child 一个 parent，无环；四路相机、雷达、IMU、设备/机器人及 aft 别名均可查询；晚加入订阅者收到全部边 | 两版 `robot-final-upright.json`、`robot-final-inverted.json` |
| 外部安装边 | probe 模拟 URDF，bridge 只发布 7 条内部边，完整树查询通过 | 两版 `robot-upright.json` |
| 机器人 IPM | 两版正装、倒装均收到非空 BGR8 图像，测试输出 80×80，frame=base_link，使用源时间戳 | 上述 JSON 的 `ipm_samples` |
| 跨版本 IPM | 正装 126 个、倒装 114 个共同源时间戳，图像 SHA256 全部相同 | `robot-cross-version.json` |
| 导出一致性 | 正装/倒装、ROS1/ROS2 的导出与广播逐值差为 0，静态树跨版本一致 | `robot-cross-version.json`；两版 `robot-final-*-extrinsics.json` |
| TF 关闭 | 动静态 TF 均为 0，原图、雷达 IMU、ROS_BODY 深度点云与机器人 IPM 继续输出 | 两版 `robot-final-disabled.json` |
| 默认设备模式 | 7 条原设备静态边、晚加入订阅和深度点云 frame 通过 | ROS1 `robot-final-device-retest.json`，ROS2 `robot-final-device.json` |
| 正式 launch | 两版 mid360 launch 实际启动，倒装 YAML、8 条 TF、图像/IMU/深度及 IPM 收发通过 | ROS1 `robot-final-launch.json`，ROS2 `robot-final-launch-retest.json` |
| 非法配置 | 缺安装文件启动明确失败；缺字段、重复键、冲突 frame、非有限值等测试通过 | ROS1 `robot-invalid-startup.log`；两版单测 |

单测使用非零平移和旋转独立验证机器人位姿杆臂；ROS 消息适配测试验证原时间戳与
原始 IMU Odometry 不变。IPM 使用坐标渐变合成图验证正装、倒装、旋转安装、相机/
安装平移、地面偏移与倾斜、非零 triple-sphere 参数、缺相机/重复相机/错 hash/
错尺寸/时间不同步和资源上限。新增组合溢出检查防止非有限静态 TF 被发布。

最初两项 launch/回归检查暴露测试环境问题，保留失败原证据：ROS1 的私有参数在
节点退出后仍保存在 master，导致设备模式继承上一轮 robot 配置；探针现在会清理
自己拥有的私有参数空间。ROS2 正式 launch 使用 SHM-only Fast DDS profile，
探针未使用同一 profile 时收不到消息；统一 RMW/profile 并配置 Docker 共享内存
后复测通过。不是用忽略错误的方式判定通过。

## 复现

在对应 Docker 内 source ROS 和 workspace 后：

```bash
# ROS1
catkin_make -j4
catkin_make -j4 run_tests_orbvi_ros_bridge
catkin_test_results
# ROS2
colcon build --packages-select orbvi_ros_bridge --cmake-args -DORBVI_ROS_BRIDGE_ROS_VERSION=2
colcon test --packages-select orbvi_ros_bridge --return-code-on-test-failure
colcon test-result --verbose
# 两版 launch 契约
python3 /source/orbvi_ros_bridge/test/test_launch_contracts.py
# 静态机器人 TF、深度与 IPM（ROS1 改为 --ros 1）
python3 /source/orbvi_ros_bridge/test/probe_tf.py --ros 2 --mode static-only \
  --tf-mode robot --mounting-config /source/orbvi_ros_bridge/config/mounting-inverted.example.yaml \
  --publish-mounting-tf --include-depth --include-ipm --host DEVICE_IP \
  --bridge-executable /path/to/orbvi_ros_bridge_node --duration 10 --output NEW_REPORT.json
# 关闭 TF 但保留传感器和 IPM：将上述 mode 改为 no-tf-sensors
orbvi_export_extrinsics --host DEVICE_IP --output NEW_EXTRINSICS.json \
  --tf-mode robot --mounting-config /absolute/path/mounting.yaml --publish-mounting-tf true
```

机器人 launch、URDF、地面平面和 RViz 示例见 [集成说明](../ROBOT_INTEGRATION.md)。
源构建新增 `libyaml-cpp-dev`，安装包依赖目标 ROS 环境及对应发行版 YAML 运行库。

## 验证边界与回滚

板端本轮没有优化位姿，**机器人动态 TF 的实板实时收发尚未完成**；已完成几何与
ROS 消息适配测试。此前设备模式的 10 分钟运动证据不当作新增机器人模式的实板
通过证据。本轮静态 TF/IPM 为短时功能验收，未完成新 IPM 工作负载的长期资源
基线、物理投影精度或用户机器人倒装实测，不据此宣称这些项目已通过。

本地交付源码、Noetic/Humble x86_64 安装产物、YAML 样例、实际设备标定生成的
外参导出样例、坐标系说明及本报告，未远程推送或向客户发布。回滚为恢复
`da34806` 版 bridge 和配套 launch；保留旧产物目录，不更改板端 runtime。
