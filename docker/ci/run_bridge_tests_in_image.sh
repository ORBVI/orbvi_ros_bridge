#!/usr/bin/env sh
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ROS_TARGET="${1:-}"
IMAGE="${2:-}"
IMAGE_ARCH="${IMAGE_ARCH:-local}"
LOG_ROOT="${LOG_ROOT:-${ROOT}/docker/ci/logs}"

if [ -z "${ROS_TARGET}" ] || [ -z "${IMAGE}" ]; then
  echo "Usage: $0 <ros1|ros2> <image>" >&2
  exit 2
fi

case "${ROS_TARGET}" in
  ros1)
    test_cmd='
set -exo pipefail
cd /workspace/project
mkdir -p ci_ws/src ci_ws/build ci_ws/devel ci_ws/install ci_ws/log
touch ci_ws/CATKIN_IGNORE ci_ws/COLCON_IGNORE
ln -s /workspace/project /workspace/project/ci_ws/src/orbvi_ros_bridge
source /opt/ros/noetic/setup.bash
cd /workspace/project/ci_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_orbvi_ros_bridge
catkin_test_results
'
    ;;
  ros2)
    test_cmd='
set -exo pipefail
cd /workspace/project
mkdir -p ci_ws/src ci_ws/build ci_ws/install ci_ws/log
touch ci_ws/CATKIN_IGNORE ci_ws/COLCON_IGNORE
ln -s /workspace/project /workspace/project/ci_ws/src/orbvi_ros_bridge
source /opt/ros/foxy/setup.bash
cd /workspace/project/ci_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select orbvi_ros_bridge
colcon test-result --verbose
'
    ;;
  *)
    echo "ERROR: unsupported ROS target: ${ROS_TARGET}" >&2
    exit 2
    ;;
esac

container_id="$(docker create "${IMAGE}" /bin/bash -lc "${test_cmd}")"

cleanup() {
  docker rm -f "${container_id}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker cp "${ROOT}/." "${container_id}:/workspace/project"

set +e
docker start -a "${container_id}"
status="$?"
set -e

log_dir="${LOG_ROOT}/${ROS_TARGET}-${IMAGE_ARCH}"
mkdir -p "${log_dir}"
docker cp "${container_id}:/workspace/project/ci_ws" "${log_dir}/" >/dev/null 2>&1 || true

exit "${status}"
