#!/usr/bin/env bash
# Build and test orbvi_ros_bridge in a colcon workspace whose src/ links back to
# the repository. Fails when no packages are discovered or no tests are recorded,
# so a misconfigured workspace cannot pass silently.
set -eo pipefail

WS="${1:-}"
if [ -z "${WS}" ]; then
  echo "Usage: $0 <colcon workspace>" >&2
  exit 2
fi
cd "${WS}"

# The workspace root carries COLCON_IGNORE so src/orbvi_ros_bridge (a symlink to
# the repository) cannot recurse into ci_ws; colcon must therefore scan src only.
packages="$(colcon list --base-paths src --names-only)"
echo "Discovered colcon packages:"
echo "${packages}"
for package in livox_ros_driver2 orbvi_ros_bridge; do
  if ! grep -qx "${package}" <<<"${packages}"; then
    echo "ERROR: colcon did not discover ${package} under ${WS}/src" >&2
    exit 1
  fi
done

colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths src --packages-select orbvi_ros_bridge \
  --event-handlers console_cohesion+ --ctest-args -V

set +e
results="$(colcon test-result --test-result-base build/orbvi_ros_bridge --verbose)"
status="$?"
set -e
echo "${results}"
if [ "${status}" -ne 0 ]; then
  exit "${status}"
fi

test_count="$(sed -n 's/^Summary: \([0-9][0-9]*\) tests.*/\1/p' <<<"${results}" | tail -n 1)"
if [ "${test_count:-0}" -eq 0 ]; then
  echo "ERROR: colcon test-result recorded 0 tests for orbvi_ros_bridge" >&2
  exit 1
fi
