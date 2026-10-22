# 4VL 板端 deb 安装与使用

本目录提供 ORBVI 4VL 板端应用运行包。板端运行相机、深度、Omni-LIVO 和 SDK Server；
用户电脑上的 ROS bridge 通过网络读取 SDK 数据，再发布 ROS topic 与 TF。

## 包信息

| 项目 | 值 |
| --- | --- |
| 文件 | `orbvi-runtime_2026.10.21+dev.db44d767_arm64.deb` |
| SDK / runtime 源码 commit | `db44d767b704bfff3ff1cde67814725e265f4b88` |
| Debian 包名 / 版本 | `orbvi-runtime` / `2026.10.21+dev.db44d767` |
| 目标设备 | ORBVI 4VL，RK3588，Ubuntu 20.04 ARM64 配套基础镜像 |
| Release profile | `production-4vl`，含板端 ROS1 组件 |
| SHA256 | `3d2c8571e1c0bd68250aa4e35bc96ebd44e3127134c657be0735c378a08151fa` |

这是应用更新包，不更新 Kernel、DTB 或 MCU，也不直接用于 OTA 上传。
安装目标必须符合上表；包声明的 ROS Noetic 等板端依赖需已由配套镜像提供。
客户端可以使用本仓库的 ROS1 或 ROS2 bridge。

## 1. 主机：校验并上传

从仓库根目录执行。修改 `board_host` 和 `board_user` 为实际设备信息；
示例地址为直连设备地址，SSH 和 sudo 使用设备账号凭据。

```bash
cd third_party/orbvi_sdk/deb
deb=orbvi-runtime_2026.10.21+dev.db44d767_arm64.deb
board_host=192.168.3.100
board_user=orbvi
target="$board_user@$board_host"

printf '%s  %s\n' \
  3d2c8571e1c0bd68250aa4e35bc96ebd44e3127134c657be0735c378a08151fa \
  "$deb" | sha256sum -c -
dpkg-deb -f "$deb" Package Version Architecture Depends
scp "$deb" "$target:/tmp/"
ssh -t "$target"
```

SHA256 必须显示 `OK`。最后一条命令进入板端终端，下一节在该终端执行。

## 2. 板端：备份、安装、重启

先核对系统和剩余空间：`uname -m` 应为 `aarch64`，系统为 Ubuntu 20.04，
安装分区建议至少保留 1 GiB 可用空间。

```bash
uname -m
cat /etc/os-release
df -h / /opt /tmp
```

**安装会清理旧 release。请提前保留可用的旧版 deb，并备份本机配置及标定。**
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

在板端重新核对已上传文件，然后安装：

```bash
printf '%s  %s\n' \
  3d2c8571e1c0bd68250aa4e35bc96ebd44e3127134c657be0735c378a08151fa \
  /tmp/orbvi-runtime_2026.10.21+dev.db44d767_arm64.deb | sha256sum -c -
sudo dpkg -i /tmp/orbvi-runtime_2026.10.21+dev.db44d767_arm64.deb
dpkg-query -W -f='${Status}\n${Version}\n${Architecture}\n' orbvi-runtime
```

只有校验通过、安装命令成功，且查询结果为 `install ok installed`、
`2026.10.21+dev.db44d767`、`arm64` 后才执行：

```bash
sudo reboot
```

若安装提示缺依赖或配置失败，保留终端日志并先解决安装问题，不要直接重启。
重启时 SSH 断开，等待设备恢复联网后再继续。

## 3. 主机：确认板端实际运行版本

在第 1 节的主机终端执行；若换了终端，先重新设置 `board_host`、`board_user`、`target`。

```bash
ssh "$target" 'dpkg-query -W orbvi-runtime; cat /data/orbvi/version.json'
ssh "$target" 'readlink -f /opt/orbvi/current; cat /opt/orbvi/current/SOURCE_COMMIT'
ssh "$target" 'systemctl is-active orbvi_camera_pipeline_native.service orbvi_timesync.service orbvi_hsnet_disparity.service orbvi_mid360.service orbvi_omni_livo.service orbvi_sdk_server.service'
curl --noproxy '*' -fsS --max-time 10 "http://$board_host:18088/v1/device/info"
curl --noproxy '*' -fsS --max-time 10 "http://$board_host:18088/v1/calibration"
```

运行版本的 `source_commit` 应等于上表 commit，服务应均为 `active`，
SDK 接口应可访问，标定状态应为 `READY`。
异常时检查板端日志：

```bash
ssh "$target" 'journalctl -b -u orbvi_sdk_server.service -u orbvi_omni_livo.service -n 100 --no-pager'
```

## 4. 主机：启动 ROS bridge

先按仓库 README 的[依赖安装与编译步骤](../../../README.zh-CN.md#安装依赖)
构建主机工作空间；安装板端 deb 不会更新主机的 bridge 或 SDK 库。
以下两种方式选一种，命令在对应 ROS 工作空间根目录执行。

ROS1 Noetic：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch orbvi_ros_bridge orbvi_ros_bridge.launch \
  host:=192.168.3.100 publish_vio_tf:=true publish_sensor_tf:=true
```

ROS2（将 `humble` 替换为主机已安装的 ROS2 发行版）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch orbvi_ros_bridge orbvi_ros_bridge.launch.py \
  host:=192.168.3.100 publish_vio_tf:=true publish_sensor_tf:=true
```

将 `host` 改成实际板端地址。相同设备的对应 TF 只保留一个发布实例；
机器人安装配置、TF 与 IPM 参数见[机器人接入说明](../../../docs/ROBOT_INTEGRATION.md)。
机器人 GPU IPM 还需要该文档规定的配套板端协议与功能，不能仅凭 deb 安装成功认定兼容。

在另一个已加载相同工作空间的 ROS2 终端检查收流：

```bash
ros2 topic hz /orbvi/vio/odometry
ros2 topic hz /orbvi/depth/points
ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable --once
```

前两条命令观察数秒后按 Ctrl+C，再执行下一条。ROS1 可分别使用
`rostopic hz /orbvi/vio/odometry`、`rostopic hz /orbvi/depth/points`、
`rostopic echo -n 1 /tf_static`。

## 回滚与验证范围

回滚时在板端重新安装提前保存的、与设备配套的旧版 deb，确认安装成功后重启，
再核对运行版本与服务；根据旧版要求恢复本机备份配置。不能依赖
`/opt/orbvi/previous`，因为该 factory 包安装时会删除旧 release。
客户端 bridge 如需回滚，恢复相应旧版本并重新编译。

该 deb 的交付记录为：已完成构建和容器验证，尚未完成该包的实板安装与全流验收。
本次入库与文档更新只核对包身份及 SHA256，没有安装或升级设备。
