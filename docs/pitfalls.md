# 开发踩坑记录

## 2026-07-28：本地 CI 脚本未发现 ROS 包

`docker/ci/run_bridge_tests_in_image.sh` 会把仓库根目录链接到容器工作区的
`src/orbvi_ros_bridge`，但实际 ROS `package.xml` 位于仓库内层
`orbvi_ros_bridge/`。因此 `colcon` 可能显示 `0 packages finished`，脚本仍以成功
状态退出，形成假阳性。

本次 IPM 改动的本地验收直接将以下目录链接到临时容器工作区：

- `orbvi_ros_bridge/orbvi_ros_bridge`
- `orbvi_ros_bridge/livox_ros_driver2`

验收时必须确认日志中明确出现 `orbvi_ros_bridge` 构建完成以及非零测试数量，不能只
检查脚本退出码。
