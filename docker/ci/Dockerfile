ARG BASE_IMAGE=harbor.huanshizhineng.com:18803/dockerhub/ubuntu:20.04
FROM ${BASE_IMAGE}

ARG BASE_IMAGE
ARG APT_MIRROR=aliyun
ARG ROS_APT_MIRROR=aliyun
ARG ROS_VERSION=1
ARG ORBVI_ROS_BRIDGE_GIT_SHA=unknown
ARG ORBVI_ROS_BRIDGE_GIT_TREE_STATE=unknown

LABEL org.opencontainers.image.title="ORBVI ROS Bridge CI"
LABEL org.opencontainers.image.description="Multi-architecture ROS build and test image for ORBVI ROS Bridge"
LABEL org.opencontainers.image.revision="${ORBVI_ROS_BRIDGE_GIT_SHA}"
LABEL org.opencontainers.image.base.name="${BASE_IMAGE}"
LABEL com.huanshi.orbvi-ros-bridge.git-tree-state="${ORBVI_ROS_BRIDGE_GIT_TREE_STATE}"
LABEL com.huanshi.orbvi-ros-bridge.ros-version="${ROS_VERSION}"
LABEL com.huanshi.orbvi-ros-bridge.apt-mirror="${APT_MIRROR}"
LABEL com.huanshi.orbvi-ros-bridge.ros-apt-mirror="${ROS_APT_MIRROR}"

ENV DEBIAN_FRONTEND=noninteractive

RUN set -eux; \
  . /etc/os-release; \
  ubuntu_suite="${VERSION_CODENAME:-focal}"; \
  arch="$(dpkg --print-architecture)"; \
  case "${APT_MIRROR}" in \
    aliyun) \
      if [ "${arch}" = "amd64" ]; then \
        ubuntu_mirror="http://mirrors.aliyun.com/ubuntu"; \
      else \
        ubuntu_mirror="http://mirrors.aliyun.com/ubuntu-ports"; \
      fi; \
      ;; \
    tuna) \
      if [ "${arch}" = "amd64" ]; then \
        ubuntu_mirror="http://mirrors.tuna.tsinghua.edu.cn/ubuntu"; \
      else \
        ubuntu_mirror="http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports"; \
      fi; \
      ;; \
    official) \
      if [ "${arch}" = "amd64" ]; then \
        ubuntu_mirror="http://archive.ubuntu.com/ubuntu"; \
        ubuntu_security_mirror="http://security.ubuntu.com/ubuntu"; \
      else \
        ubuntu_mirror="http://ports.ubuntu.com/ubuntu-ports"; \
      fi; \
      ;; \
    http://*|https://*) \
      ubuntu_mirror="${APT_MIRROR%/}"; \
      ;; \
    *) \
      echo "Unsupported APT_MIRROR: ${APT_MIRROR}" >&2; \
      exit 1; \
      ;; \
  esac; \
  ubuntu_security_mirror="${ubuntu_security_mirror:-${ubuntu_mirror}}"; \
  { \
    echo "deb ${ubuntu_mirror}/ ${ubuntu_suite} main restricted universe multiverse"; \
    echo "deb ${ubuntu_mirror}/ ${ubuntu_suite}-updates main restricted universe multiverse"; \
    echo "deb ${ubuntu_mirror}/ ${ubuntu_suite}-backports main restricted universe multiverse"; \
    echo "deb ${ubuntu_security_mirror}/ ${ubuntu_suite}-security main restricted universe multiverse"; \
  } > /etc/apt/sources.list

RUN rm -f /etc/apt/apt.conf.d/docker-clean

RUN --mount=type=cache,id=orbvi-ros-bridge-ci-apt-cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,id=orbvi-ros-bridge-ci-apt-lists,target=/var/lib/apt/lists,sharing=locked \
  set -eux; \
  apt-get -o Acquire::Retries=3 update; \
  apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    dirmngr \
    file \
    gnupg2 \
    libboost-dev \
    lsb-release \
    libgtest-dev \
    libopencv-dev \
    locales \
    pkg-config \
    python3-pip \
    python3-setuptools \
    python3-yaml \
    tzdata; \
  locale-gen en_US.UTF-8; \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; \
  ln -fs /usr/share/zoneinfo/UTC /etc/localtime; \
  dpkg-reconfigure --frontend noninteractive tzdata

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN --mount=type=cache,id=orbvi-ros-bridge-ci-apt-cache,target=/var/cache/apt,sharing=locked \
  --mount=type=cache,id=orbvi-ros-bridge-ci-apt-lists,target=/var/lib/apt/lists,sharing=locked \
  set -eux; \
  case "${ROS_VERSION}" in \
    1) \
      ros_suite="ros"; \
      ros_distro="noetic"; \
      case "${ROS_APT_MIRROR}" in \
        aliyun) ros_mirror="http://mirrors.aliyun.com/ros/ubuntu" ;; \
        tuna) ros_mirror="http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu" ;; \
        official) ros_mirror="http://packages.ros.org/ros/ubuntu" ;; \
        http://*|https://*) ros_mirror="${ROS_APT_MIRROR%/}" ;; \
        *) echo "Unsupported ROS_APT_MIRROR: ${ROS_APT_MIRROR}" >&2; exit 1 ;; \
      esac; \
      ;; \
    2) \
      ros_suite="ros2"; \
      ros_distro="foxy"; \
      case "${ROS_APT_MIRROR}" in \
        aliyun) ros_mirror="http://mirrors.aliyun.com/ros2/ubuntu" ;; \
        tuna) ros_mirror="http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu" ;; \
        official) ros_mirror="http://packages.ros.org/ros2/ubuntu" ;; \
        http://*|https://*) ros_mirror="${ROS_APT_MIRROR%/}" ;; \
        *) echo "Unsupported ROS_APT_MIRROR: ${ROS_APT_MIRROR}" >&2; exit 1 ;; \
      esac; \
      ;; \
    *) \
      echo "Unsupported ROS_VERSION: ${ROS_VERSION}" >&2; \
      exit 1; \
      ;; \
  esac; \
  curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -; \
  echo "deb ${ros_mirror}/ $(lsb_release -sc) main" > "/etc/apt/sources.list.d/${ros_suite}-latest.list"; \
  apt-get -o Acquire::Retries=3 update; \
  if [ "${ROS_VERSION}" = "1" ]; then \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      python3-catkin-tools \
      ros-noetic-message-generation \
      ros-noetic-nav-msgs \
      ros-noetic-ros-base \
      ros-noetic-sensor-msgs; \
  else \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      python3-colcon-common-extensions \
      ros-foxy-ament-cmake-gtest \
      ros-foxy-nav-msgs \
      ros-foxy-ros-base \
      ros-foxy-rosidl-default-generators \
      ros-foxy-sensor-msgs; \
  fi; \
  mkdir -p /opt/huanshi/orbvi-ros-bridge; \
  dpkg-query -W -f='${binary:Package}\t${Version}\t${Architecture}\n' \
    | LC_ALL=C sort \
    > /opt/huanshi/orbvi-ros-bridge/dpkg-manifest.txt; \
  { \
    printf 'BASE_IMAGE=%s\n' "${BASE_IMAGE}"; \
    printf 'APT_MIRROR=%s\n' "${APT_MIRROR}"; \
    printf 'ROS_APT_MIRROR=%s\n' "${ROS_APT_MIRROR}"; \
    printf 'ROS_VERSION=%s\n' "${ROS_VERSION}"; \
    printf 'ROS_DISTRO=%s\n' "${ros_distro}"; \
    printf 'ORBVI_ROS_BRIDGE_GIT_SHA=%s\n' "${ORBVI_ROS_BRIDGE_GIT_SHA}"; \
    printf 'ORBVI_ROS_BRIDGE_GIT_TREE_STATE=%s\n' "${ORBVI_ROS_BRIDGE_GIT_TREE_STATE}"; \
    printf 'DPKG_MANIFEST_PATH=%s\n' "/opt/huanshi/orbvi-ros-bridge/dpkg-manifest.txt"; \
  } > /opt/huanshi/orbvi-ros-bridge/build-metadata.env

ENV DEBIAN_FRONTEND=
WORKDIR /workspace
CMD ["/bin/bash"]
