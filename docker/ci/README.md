# ORBVI ROS Bridge CI Images

This directory builds the Harbor CI images used by GitLab jobs. The images
preinstall Ubuntu 20.04 build tools, OpenCV, GTest and the matching ROS
distribution so normal bridge CI jobs do not install large apt dependency sets
at runtime.

Default image repository:

```bash
harbor.huanshizhineng.com:18803/omnisense/orbvi_ros_bridge_ci
```

The build script creates two image tags for the current git short SHA when
building multi-platform images manually:

```text
ros1-focal-<sha>
ros2-focal-<sha>
```

Each tag is a multi-platform manifest for:

```text
linux/amd64
linux/arm64
```

GitLab CI builds native per-runner images instead:

```text
ros1-focal-<sha>-amd64
ros2-focal-<sha>-amd64
ros1-focal-<sha>-arm64
ros2-focal-<sha>-arm64
```

The `ubuntu` runner builds and tests `linux/amd64`; the `orbvi` runner builds
and tests `linux/arm64`.

## Build Locally

Build a single local image for quick checks:

```bash
PLATFORMS=linux/arm64 IMAGE_ARCH=arm64 ROS_TARGETS=ros1 docker/ci/build.sh
PLATFORMS=linux/amd64 IMAGE_ARCH=amd64 ROS_TARGETS=ros2 docker/ci/build.sh
```

Build and push the formal multi-platform images:

```bash
printf '%s' "$HARBOR_PASSWORD" | docker login harbor.huanshizhineng.com:18803 \
  -u "$HARBOR_USERNAME" --password-stdin

PUSH=1 docker/ci/build.sh
```

If an image is built under a temporary local name, retag it into the Harbor
`omnisense` project before pushing:

```bash
docker tag SOURCE_IMAGE[:TAG] harbor.huanshizhineng.com:18803/omnisense/REPOSITORY[:TAG]
docker push harbor.huanshizhineng.com:18803/omnisense/REPOSITORY[:TAG]
```

The script refuses formal builds from a dirty worktree. For temporary local
debugging only:

```bash
ALLOW_DIRTY=1 PLATFORMS=linux/arm64 ROS_TARGETS=ros1 docker/ci/build.sh
```

## GitLab CI

`.gitlab-ci.yml` builds and pushes four native images first. The ROS1 and ROS2
test jobs then consume the exact commit and platform tag that was just pushed.

GitLab CI variables required for pushing to Harbor:

```text
HARBOR_USERNAME
HARBOR_PASSWORD
```

Alternatively, set a Docker-compatible `DOCKER_AUTH_CONFIG` variable. The Docker
build jobs fail fast if neither credential source is available.

The DockerHub base image is pulled through the company Harbor proxy:

```text
harbor.huanshizhineng.com:18803/dockerhub/ubuntu:20.04
```

GitLab CI also pulls Docker CLI and BuildKit through the same proxy:

```text
harbor.huanshizhineng.com:18803/dockerhub/docker:27-cli
harbor.huanshizhineng.com:18803/dockerhub/moby/buildkit:buildx-stable-1
```
