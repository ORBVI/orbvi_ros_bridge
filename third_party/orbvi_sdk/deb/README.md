# 4V / 4VL 板端 deb 安装与使用

本目录提供 ORBVI 4V 与 4VL 板端应用运行包。板端运行相机、深度、定位（4V 为 VINS，
4VL 为 MID360 + Omni-LIVO）和 SDK Server；用户电脑上的 ROS bridge 通过网络读取
SDK 数据，再发布 ROS topic 与 TF。

> **升级提示：当前版本的 bridge 必须配合本目录的 `20261105.1+gb580d06d2208` deb 使用。**
> 本版 bridge 新增了定位（`/orbvi/localization/*`）、Omni-LIVO 彩色点云、SI 单位 IMU
> 和校正 JPEG 等数据流，并与同一 commit（`b580d06d`）的 Host SDK 一起编译。
> 板端仍为旧版 runtime（例如 `2026.10.21+dev.db44d767`）时，相关 topic 可能缺失或
> 数据异常。请先按下文把板端升级到对应型号的新 deb，再重新编译并启动 bridge。

## 包信息

两个包来自同一 SDK / runtime 源码 commit，仅硬件 profile 不同。**请按设备型号选择，
不要把 4V 包装到 4VL 设备上，反之亦然。**

| 项目 | 4V（四目，无激光雷达） | 4VL（四目 + MID360） |
| --- | --- | --- |
| 文件 | `orbvi-runtime_20261105.1+gb580d06d2208.4v.reset_arm64.deb` | `orbvi-runtime_20261105.1+gb580d06d2208.4vl.reset_arm64.deb` |
| Debian 包名 / 版本 | `orbvi-runtime` / `20261105.1+gb580d06d2208.4v.reset` | `orbvi-runtime` / `20261105.1+gb580d06d2208.4vl.reset` |
| Release profile | `production-4v` | `production-4vl` |
| 板端定位 | VINS | Omni-LIVO |
| SHA256 | `e6ab950e3dd09e0b0472a664305b31e5301be05c0c2e925ee5fba93f6485eef7` | `408599d2292a0aaa76e5fcd546fa589b6c7e1b8a577f920bf56cd6217025d27d` |

公共信息：

| 项目 | 值 |
| --- | --- |
| SDK / runtime 源码 commit | `b580d06d2208ec2487182f2c4a7638d0ad0f3351` |
| 目标设备 | ORBVI 4V / 4VL，RK3588，Ubuntu 20.04 ARM64 配套基础镜像，含板端 ROS1 组件 |
| 配套主机 bridge | 本仓库当前版本（Host SDK 同步自 `b580d06d`） |

这是应用更新包（factory runtime bootstrap），不更新 Kernel、DTB 或 MCU，
也不直接用于 OTA 上传。文件名中的 `reset` 表示安装会写入默认 runtime 配置并清理旧
release，因此安装前务必按第 2 节备份。安装目标必须符合上表；包声明的 ROS Noetic
等板端依赖需已由配套镜像提供。客户端可以使用本仓库的 ROS1 或 ROS2 bridge。

## 1. 主机：校验并上传

从仓库根目录执行。按设备型号设置 `model`（`4v` 或 `4vl`），并修改 `board_host`
和 `board_user` 为实际设备信息；示例地址为直连设备地址，SSH 和 sudo 使用设备账号凭据。

```bash
cd third_party/orbvi_sdk/deb
model=4vl   # 4V 设备改为 4v
case "$model" in
  4v)  sha=e6ab950e3dd09e0b0472a664305b31e5301be05c0c2e925ee5fba93f6485eef7 ;;
  4vl) sha=408599d2292a0aaa76e5fcd546fa589b6c7e1b8a577f920bf56cd6217025d27d ;;
esac
deb=orbvi-runtime_20261105.1+gb580d06d2208.${model}.reset_arm64.deb
board_host=192.168.3.100
board_user=orbvi
target="$board_user@$board_host"

printf '%s  %s\n' "$sha" "$deb" | sha256sum -c -
dpkg-deb -f "$deb" Package Version Architecture Depends
scp "$deb" "$target:/tmp/"
echo "model=$model sha=$sha deb=$deb"
ssh -t "$target"
```

SHA256 必须显示 `OK`。记下最后打印的 `model`、`sha`、`deb` 三个值，
最后一条命令进入板端终端，下一节在该终端执行。

## 2. 板端：备份、安装、重启

先核对系统和剩余空间：`uname -m` 应为 `aarch64`，系统为 Ubuntu 20.04，
安装分区建议至少保留 1 GiB 可用空间。

```bash
uname -m
cat /etc/os-release
df -h / /opt /tmp
cat /data/orbvi/version.json   # 确认当前设备的 hardware_profile 与要安装的型号一致
```

**安装会清理旧 release 并写入默认配置。请提前保留可用的旧版 deb，并备份本机配置及标定。**
以下备份覆盖默认路径；若设备配置了额外标定路径，也应加入备份。
在板端 Bash 终端逐步执行，确认备份成功后再安装：

```bash
backup_dir="$HOME/orbvi-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -m 700 "$backup_dir"
paths=()
for path in etc/default/orbvi etc/orbvi etc/systemd/system var/lib/board-calib data/orbvi/config; do
  if [ -e "/$path" ]; then paths+=("$path"); fi
done
sudo tar -czf "$backup_dir/config-before.tar.gz" -C / "${paths[@]}"
sudo chown "$(id -u):$(id -g)" "$backup_dir/config-before.tar.gz"
chmod 600 "$backup_dir/config-before.tar.gz"
readlink -f /opt/orbvi/current > "$backup_dir/release-before.txt"
dpkg-query -W orbvi-runtime > "$backup_dir/package-before.txt"
printf '备份目录：%s\n' "$backup_dir"
```

首次安装时 `dpkg-query` 可能提示未安装该包；已有设备升级应能记录旧版本。
记下备份目录，并从另一主机终端用 `scp -r 用户名@板端IP:备份目录 ./`
将备份保存到主机。不要用另一台设备的标定替换本机标定。

在板端用第 1 节记下的值重新设置变量，核对已上传文件，然后安装：

```bash
model=4vl   # 与第 1 节一致
sha=<第 1 节打印的 sha>
deb=/tmp/orbvi-runtime_20261105.1+gb580d06d2208.${model}.reset_arm64.deb

printf '%s  %s\n' "$sha" "$deb" | sha256sum -c -
sudo dpkg -i "$deb"
dpkg-query -W -f='${Status}\n${Version}\n${Architecture}\n' orbvi-runtime
```

只有校验通过、安装命令成功，且查询结果为 `install ok installed`、
`20261105.1+gb580d06d2208.<model>.reset`、`arm64` 后才执行：

```bash
sudo reboot
```

若安装提示缺依赖或配置失败，保留终端日志并先解决安装问题，不要直接重启。
重启时 SSH 断开，等待设备恢复联网后再继续。

## 3. 主机：确认板端实际运行版本

在第 1 节的主机终端执行；若换了终端，先重新设置 `model`、`board_host`、`board_user`、`target`。

```bash
ssh "$target" 'dpkg-query -W orbvi-runtime; cat /data/orbvi/version.json'
ssh "$target" 'readlink -f /opt/orbvi/current; cat /opt/orbvi/current/SOURCE_COMMIT'
common="orbvi_camera_pipeline_native.service orbvi_timesync.service orbvi_hsnet_disparity.service orbvi_sdk_server.service"
case "$model" in
  4v)  services="$common orbvi_vins.service" ;;
  4vl) services="$common orbvi_mid360.service orbvi_omni_livo.service" ;;
esac
ssh "$target" "systemctl is-active $services"
curl --noproxy '*' -fsS --max-time 10 "http://$board_host:18088/v1/device/info"
curl --noproxy '*' -fsS --max-time 10 "http://$board_host:18088/v1/calibration"
```

`version.json` 中 `source_commit` 应为 `b580d06d2208ec2487182f2c4a7638d0ad0f3351`，
`release_profile` 应为 `production-4v` 或 `production-4vl`（与设备型号一致）；
服务应均为 `active`，SDK 接口应可访问，标定状态应为 `READY`。
异常时检查板端日志（4V 将 `orbvi_omni_livo.service` 换成 `orbvi_vins.service`）：

```bash
ssh "$target" 'journalctl -b -u orbvi_sdk_server.service -u orbvi_omni_livo.service -n 100 --no-pager'
```

## 4. 主机：更新并启动 ROS bridge

板端升级后，主机侧也必须使用本仓库当前版本的 bridge：拉取最新代码后，先按仓库 README 的
[依赖安装与编译步骤](../../../README.zh-CN.md#安装依赖)**清理并重新编译**主机工作空间
（Host SDK 头文件与库已同步更新，旧编译产物不能复用）。安装板端 deb 不会更新主机的
bridge 或 SDK 库。以下两种方式选一种，命令在对应 ROS 工作空间根目录执行；
4V 设备将 launch 文件中的 `4vl` 换成 `4v`。

ROS1 Noetic：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch
```

ROS2（将 `humble` 替换为主机已安装的 ROS2 发行版）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_4vl.launch.py
```

板端地址默认为 `192.168.3.100`，不同时追加 `host:=<板端 IP>`。相同设备的对应 TF 只保留一个发布实例；
机器人安装配置、TF 与 IPM 参数见[机器人接入说明](../../../docs/ROBOT_INTEGRATION.md)。
机器人 GPU IPM 还需要该文档规定的配套板端协议与功能，不能仅凭 deb 安装成功认定兼容。

在另一个已加载相同工作空间的 ROS2 终端检查收流：

```bash
ros2 topic hz /orbvi/localization/odometry
ros2 topic hz /orbvi/depth/points
ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable --once
```

前两条命令观察数秒后按 Ctrl+C，再执行下一条。ROS1 可分别使用
`rostopic hz /orbvi/localization/odometry`、`rostopic hz /orbvi/depth/points`、
`rostopic echo -n 1 /tf_static`。若 `/orbvi/localization/odometry` 没有数据，
优先确认第 3 节的板端版本是否已是 `b580d06d`。

## 5. 主机：打开 RViz 可视化

bridge 保持运行，**另开一个终端**并加载相同工作空间（同第 4 节的两条 `source`），然后执行：

```bash
# ROS1
roslaunch orbvi_ros_bridge orbvi_ros_bridge_rviz.launch profile:=mid360       # 4VL
roslaunch orbvi_ros_bridge orbvi_ros_bridge_rviz.launch profile:=visual       # 4V
# ROS2
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_rviz.launch.py profile:=mid360  # 4VL
ros2 launch orbvi_ros_bridge orbvi_ros_bridge_rviz.launch.py profile:=visual  # 4V
```

`mid360` 以 `camera_init` 为固定坐标系，显示累积彩色点云和定位轨迹；`visual` 以
`orbvi_link` 为固定坐标系，显示实时深度点云，不依赖定位。RViz 提示 Fixed Frame
不存在时改用 `profile:=visual`。详见[查看数据](../../../docs/VIEWING.zh-CN.md)。

## 回滚与验证范围

回滚时在板端重新安装提前保存的、与设备配套的旧版 deb，确认安装成功后重启，
再核对运行版本与服务；根据旧版要求恢复本机备份配置。不能依赖
`/opt/orbvi/previous`，因为该 factory 包安装时会删除旧 release。
板端回滚到旧版后，客户端 bridge 也需恢复到与之配套的旧版本并重新编译。

该版本 deb 已完成实板安装，并与当前版本 bridge 配合验证通过。包身份（`control`、
`version.json`、`SOURCE_COMMIT`）及 SHA256 已与上表核对一致。
