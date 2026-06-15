# ORBVI ROS Bridge CI Images

This directory builds the Docker images used by maintainer CI. The images
preinstall Ubuntu 20.04 build tools, OpenCV, GTest and the matching ROS
distribution so bridge test jobs do not install large apt dependency sets at
runtime.

The image repository and base image are controlled by `IMAGE_REPOSITORY` and
`BASE_IMAGE`. Organization CI may override both values to use an internal
registry or mirror.

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

Build and push formal multi-platform images:

```bash
IMAGE_REPOSITORY=<registry>/<project>/orbvi_ros_bridge_ci \
  PUSH=1 docker/ci/build.sh
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

`.gitlab-ci.yml` uses different flows for branch checks and release-line
pushes:

- Merge request branches run the `docker:*:check` jobs. Each job builds its
  native platform image with `PUSH=0`, then runs the ROS bridge tests inside
  that local image with `docker/ci/run_bridge_tests_in_image.sh`.
- Default-branch and tag pipelines run the push jobs. They build and push the
  exact commit/platform image tag, then the ROS1 and ROS2 test jobs consume the
  pushed image.

Push jobs require registry credentials in CI. Branch check jobs continue
without Docker login because they run with `PUSH=0`.
