# ORBVI ROS Bridge CI Images

This directory builds the Docker images used by maintainer CI. The images
preinstall Ubuntu 20.04 build tools, OpenCV, GTest and the matching ROS
distribution so bridge test jobs do not install large apt dependency sets at
runtime.

The image repository and base image are controlled by `IMAGE_REPOSITORY` and
`BASE_IMAGE`. Organization CI may override both values to use an internal
registry or mirror.

Normal GitLab pipelines consume prebuilt images directly and only compile/test
the current ROS packages. The image tag suffix is selected by
`CI_IMAGE_TAG_SUFFIX` in `.gitlab-ci.yml`:

```text
ros1-focal-<CI_IMAGE_TAG_SUFFIX>-amd64
ros1-focal-<CI_IMAGE_TAG_SUFFIX>-arm64
ros2-focal-<CI_IMAGE_TAG_SUFFIX>-amd64
ros2-focal-<CI_IMAGE_TAG_SUFFIX>-arm64
```

The build script can still create commit-specific tags for local experiments or
one-off image snapshots. GitLab maintenance jobs set `IMAGE_TAG_SUFFIX=ci` so
stable `*-ci-*` tags can be refreshed manually when needed.

Manual multi-platform builds can target:

```text
linux/amd64
linux/arm64
```

GitLab CI builds native per-runner images:

```text
ros1-focal-<tag>-amd64
ros2-focal-<tag>-amd64
ros1-focal-<tag>-arm64
ros2-focal-<tag>-arm64
```

The `ubuntu` runner builds and tests `linux/amd64`; the `orbvi` runner builds
and tests `linux/arm64`.

## Build Locally

Build a single local image for quick checks:

```bash
PLATFORMS=linux/arm64 IMAGE_ARCH=arm64 ROS_TARGETS=ros1 docker/ci/build.sh
PLATFORMS=linux/amd64 IMAGE_ARCH=amd64 ROS_TARGETS=ros2 docker/ci/build.sh
```

Build and push the maintained CI image tags:

```bash
IMAGE_REPOSITORY=<registry>/<project>/orbvi_ros_bridge_ci \
  IMAGE_TAG_SUFFIX=ci PUSH=1 docker/ci/build.sh
```

If an image is built under a temporary local name, retag it into the target
repository before pushing:

```bash
docker tag SOURCE_IMAGE[:TAG] <registry>/<project>/REPOSITORY[:TAG]
docker push <registry>/<project>/REPOSITORY[:TAG]
```

The script refuses formal builds from a dirty worktree. For temporary local
debugging only:

```bash
ALLOW_DIRTY=1 PLATFORMS=linux/arm64 ROS_TARGETS=ros1 docker/ci/build.sh
```

## GitLab CI

`.gitlab-ci.yml` separates ROS package checks from CI image maintenance:

- Merge request and branch pipelines run the ROS1/ROS2 build/test jobs against
  the stable `*-ci-*` images. They do not rebuild Docker images.
- Docker jobs are manual maintenance jobs. Run them only when the CI Dockerfile
  or base dependency set changes.

Docker refresh jobs require registry credentials in CI.
