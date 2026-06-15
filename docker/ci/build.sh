#!/usr/bin/env sh
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
DOCKERFILE="${ROOT}/docker/ci/Dockerfile"
IMAGE_REPOSITORY="${IMAGE_REPOSITORY:-harbor.huanshizhineng.com:18803/omnisense/orbvi_ros_bridge_ci}"
BASE_IMAGE="${BASE_IMAGE:-harbor.huanshizhineng.com:18803/dockerhub/ubuntu:20.04}"
PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
IMAGE_ARCH="${IMAGE_ARCH:-}"
ROS_TARGETS="${ROS_TARGETS:-ros1 ros2}"
PUSH="${PUSH:-0}"
ALLOW_DIRTY="${ALLOW_DIRTY:-0}"
APT_MIRROR="${APT_MIRROR:-aliyun}"
ROS_APT_MIRROR="${ROS_APT_MIRROR:-aliyun}"
BUILDKIT_PROGRESS="${BUILDKIT_PROGRESS:-plain}"
MANIFEST_DIR="${MANIFEST_DIR:-${ROOT}/docker/ci/manifests}"

if command -v git >/dev/null 2>&1 && [ -d "${ROOT}/.git" ]; then
  git_sha="$(git -C "${ROOT}" rev-parse HEAD)"
  git_short_sha="${CI_COMMIT_SHORT_SHA:-$(git -C "${ROOT}" rev-parse --short HEAD)}"
else
  git_sha="${CI_COMMIT_SHA:-unknown}"
  git_short_sha="${CI_COMMIT_SHORT_SHA:-unknown}"
fi
git_tree_state="clean"

if command -v git >/dev/null 2>&1 && [ -d "${ROOT}/.git" ]; then
  dirty_status="$(
    git -C "${ROOT}" status --porcelain --untracked-files=all -- \
      . \
      ':(exclude)docker/ci/manifests/**' \
      ':(exclude)docker/ci/logs/**'
  )"
else
  dirty_status=""
fi

if [ -n "${dirty_status}" ]; then
  git_tree_state="dirty"
  if [ "${ALLOW_DIRTY}" != "1" ]; then
    echo "ERROR: working tree has dirty or untracked files; refusing formal CI image build." >&2
    echo "Commit the Docker/CI changes first, or use ALLOW_DIRTY=1 for local debugging." >&2
    printf '%s\n' "${dirty_status}" >&2
    exit 1
  fi
fi

if [ -z "${IMAGE_TAG_SUFFIX+x}" ]; then
  IMAGE_TAG_SUFFIX="${git_short_sha}"
fi

if [ "${git_tree_state}" = "dirty" ]; then
  case "${IMAGE_TAG_SUFFIX}" in
    *dirty*) ;;
    *) IMAGE_TAG_SUFFIX="${IMAGE_TAG_SUFFIX}-dirty" ;;
  esac
fi

if [ "${PUSH}" != "1" ]; then
  case "${PLATFORMS}" in
    *,*)
      echo "ERROR: multi-platform local load is not supported. Set PUSH=1 or use a single PLATFORM." >&2
      exit 1
      ;;
  esac
fi

mkdir -p "${MANIFEST_DIR}"
metadata_file="${MANIFEST_DIR}/ci-images-${IMAGE_TAG_SUFFIX}.env"
: > "${metadata_file}"

tag_arch_suffix=""
if [ -n "${IMAGE_ARCH}" ]; then
  tag_arch_suffix="-${IMAGE_ARCH}"
fi

build_one() {
  target="$1"
  case "${target}" in
    ros1)
      ros_version="1"
      image_tag="ros1-focal-${IMAGE_TAG_SUFFIX}${tag_arch_suffix}"
      ;;
    ros2)
      ros_version="2"
      image_tag="ros2-focal-${IMAGE_TAG_SUFFIX}${tag_arch_suffix}"
      ;;
    *)
      echo "ERROR: unsupported ROS target: ${target}" >&2
      exit 1
      ;;
  esac

  set -- \
    docker buildx build \
    --platform "${PLATFORMS}" \
    --progress "${BUILDKIT_PROGRESS}" \
    --file "${DOCKERFILE}" \
    --tag "${IMAGE_REPOSITORY}:${image_tag}" \
    --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
    --build-arg "APT_MIRROR=${APT_MIRROR}" \
    --build-arg "ROS_APT_MIRROR=${ROS_APT_MIRROR}" \
    --build-arg "ROS_VERSION=${ros_version}" \
    --build-arg "ORBVI_ROS_BRIDGE_GIT_SHA=${git_sha}" \
    --build-arg "ORBVI_ROS_BRIDGE_GIT_TREE_STATE=${git_tree_state}" \
    --label "org.opencontainers.image.base.name=${BASE_IMAGE}" \
    --label "org.opencontainers.image.revision=${git_sha}" \
    --label "com.huanshi.orbvi-ros-bridge.git-tree-state=${git_tree_state}" \
    --label "com.huanshi.orbvi-ros-bridge.image-arch=${IMAGE_ARCH}" \
    --label "com.huanshi.orbvi-ros-bridge.ros-target=${target}" \
    --label "com.huanshi.orbvi-ros-bridge.apt-mirror=${APT_MIRROR}" \
    --label "com.huanshi.orbvi-ros-bridge.ros-apt-mirror=${ROS_APT_MIRROR}"

  if [ "${PUSH}" = "1" ]; then
    set -- "$@" --push
  else
    set -- "$@" --load
  fi

  set -- "$@" "${ROOT}"

  printf '[build]'
  for arg in "$@"; do
    printf ' %s' "${arg}"
  done
  printf '\n'
  "$@"

  printf '%s_IMAGE=%s:%s\n' "$(printf '%s' "${target}" | tr '[:lower:]' '[:upper:]')" "${IMAGE_REPOSITORY}" "${image_tag}" >> "${metadata_file}"
  echo "[done] ${target}: ${IMAGE_REPOSITORY}:${image_tag}"
}

for target in ${ROS_TARGETS}; do
  build_one "${target}"
done

echo "[manifest] ${metadata_file}"
cat "${metadata_file}"
