# 世界系与安装模式复核（2026-10-15）

> 本文记录修复前状态。安装 TF 真值与可选启动 odom 的后续修复见
> [TF 所有权修复验收](2026-10-15-tf-ownership-fix.md)。

本次对 bridge `f5f578a`、当前 native 源码与本地 4VL 板只读核对，未修改或重启
板端服务。另进行了 ROS1/ROS2 各 12 秒机器人动态 TF 收发验证。没有倒装设备或
重新上电，不能将本轮记录写成倒装启动实测通过。

## camera_init 与重力

当前 native 源码包含提交 `4e82ab88` 引入的 `NativeGravityAlignment`。
`ImuProcess::IMU_init` 先从均值加速度初始化重力、将姿态设为单位旋转；随后 native
在 IMU 初始化完成、正常地图更新前，将世界重力旋转到 -Z。此操作同时处理世界
姿态、位置、速度和相关协方差；每个算法实例只执行一次。配置
`uav.gravity_align_en` 的 native 默认值为 true。该实现包含倒装重力 +Z 的测试用例；
这里未重新运行 native 测试，也不将源码中的测试用例描述成新的硬件验收结果。

只读实板检查：

- 版本路径：`release-4vl-20261015-g1bae9e43-localization-r9`。
- `orbvi_omni_livo.service` 为 active/running，检查时 PID 15416。
- native 本地只读 status 返回 `world_frame="gravity-aligned-z-up-v1"`。
- 一组运行状态重力约为 `[-0.0048,-0.0144,-9.8100] m/s²`，init=0。
- 有效 YAML 未显式覆盖 gravity_align_en，使用 native 默认开启值。
- 原始证据：`build/tf-bridge-20261015/board/world-review-runtime.txt`
  （相对 `/home/xingmaker/huanshi/`）。

因此当前版本不能按“camera_init 必然保持上电 IMU 朝向”处理，也不应再单凭倒装
安装外参补一次重力旋转。旧版本或显式关闭重力对齐时，必须重新核对实际语义。
只根据安装姿态计算世界调平还隐含“启动时机器人水平”的假设，安装外参不能独立
确定任意启动倾角。

重力对齐不确定 yaw，也不会把机器人起点、地面平面自动移到原点。世界原点沿用
算法初始化参考点：地面水平时，其 z 通常也不是 0。需要起点归零或地面 z=0，应
结合首个有效机器人位姿和已知地面高度定义 odom/footprint；这与是否倒装不同。

`camera_init` 当前作为根节点不妨碍之后增加父节点。未来要求 `map → odom` 时，
可明确接入 `map → odom → camera_init → base_link` 的所有权和世界关系；当前
bridge 没有发布 map/odom。新增 odom 变换还要考虑算法代次重置，不能把一次
计算的静态关系跨所有算法重启沿用。当前版本无需为“缺少父节点”发布虚构的单位边。

## 安装外参所有权：仍有缺口

bridge 自己的机器人位姿和 IPM 使用同一份安装 YAML；在 bridge 发布安装边时，
这三处数值一致。URDF 模式则是另一份 URDF 安装参数，同时 bridge 仍读取 YAML。
当前没有 TF listener 读取 URDF 的安装边，也没有 xacro 示例或自动互斥机制。
所以 URDF 模式仍有配置不一致和重复发布的风险，这一点尚未修复。

后续统一方式应为：URDF 模式由 TF 提供静态安装关系，机器人位姿读取
`T_base_mid360_imu`，IPM 读取相同树下的 `T_base_camera`；bridge 模式从配置
发布安装边，并使用同一套解析结果。缺 TF 时等待或禁用依赖分支，不能回退另一套
隐藏安装数值。需要保持原有启动快照语义，安装变更后重启或显式重建 IPM 映射。
xacro 开关只能约束使用该 xacro 的机器人，不足以阻止外部独立发布者；验收还要
按 `/tf_static` 的 publisher GID/callerid 逐边统计，不能只看最终 tf2 树。

另外，机器人树**仍含** `mid360_imu → aft_mapped` 静态单位别名，用于兼容原始
Odometry 的 child 名称。旧动态边已撤除；别名不重复应用外参，但与“aft_mapped
已经完全移出树”不同。是否彻底移除应作为明确的兼容性调整处理。

## IPM、方位和时间

- 地面平面是显式参数，表达在 mounting.parent_frame（默认 base_link）中；没有从
  base_footprint 自动生成。倒装不再旋转一次该平面。此项已覆盖。
- camera ID 只用来匹配图像与标定；逐像素使用完整 `T_base_device * T_device_camera`
  逆变换投影，视野和融合权重由相机系中的射线确定。没有按 ID 固定方位槽位、接缝
  区域或遮挡掩膜；role 仅验证 rig 完整性与参考标定身份。安装是静态快照，所以
  映射在启动时预计算，无需每帧重算方位。正装、倒装、绕 Z 90° 的投影已有合成测试。
- 单帧 IPM 仅依赖静态外参，输出采用四路源图像中的最新采样时间，输入时差最多
  10 ms。未实现世界系多帧 BEV 累积，也没有为它提供位姿等待缓存或 IMU 高频
  位姿；不能用 latest TF 代替采样时刻变换。此项是后续功能边界。
- ROS 标准 IMU `linear_acceleration` 是比力：静止且机器人水平时，转换到
  base_link 后通常应约为 `(0,0,+g)`；估计的重力向量才是 `(0,0,-g)`。验收必须
  区分两种量。用同一标志物评估 IPM 小于一像素属于物理投影精度测试，本次未做。

## 新增实时动态 TF 验证

两版 bridge 使用 upright 示例安装（只用于组合验证，不代表实际机器人安装），
从同一板 SDK 接收真实优化位姿：

| 项目 | ROS1 Noetic | ROS2 Humble |
| --- | --- | --- |
| 匹配原始 IMU Odometry 与机器人动态 TF | 115 对 | 115 对 |
| 独立矩阵组合误差阈值 | ≤1e-9，通过 | ≤1e-9，通过 |
| 世界到各传感器最新/采样时刻查询 | 通过 | 通过 |
| 8 条完整静态边、晚加入订阅 | 通过 | 通过 |
| 图像、雷达 IMU、深度点云、机器人 IPM | 收到 | 收到 |

证据为 `build/tf-bridge-20261015/{noetic,humble}/world-review-robot.json`、
`world-review-robot.bridge.log` 和 `world-review.log`。这补上了上一轮因板端服务
failed 而未完成的机器人动态 TF 实时收发；没有替代倒装上电或运动精度测试。
