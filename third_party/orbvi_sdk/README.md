# ORBVI Host SDK Bundle

This directory vendors the public ORBVI Host SDK headers and prebuilt Linux
libraries used by the ROS bridge package.

The layout is intentionally simple for SDK users: keep headers under one
include directory, keep one library directory per Linux architecture and let
the bridge CMake pick the matching binary automatically.

## Layout

| Path | Content |
| --- | --- |
| `include/orbvi_sdk/` | Public C++ Host SDK headers |
| `libs/linux-x86_64/` | Ubuntu x86_64 / amd64 Host SDK libraries |
| `libs/linux-aarch64/` | Ubuntu aarch64 / arm64 Host SDK libraries |

The bridge CMake defaults to the matching library directory for the current
Linux architecture. Set `ORBVI_HOST_SDK_ROOT`,
`ORBVI_HOST_SDK_INCLUDE_DIR`/`ORBVI_HOST_SDK_LIBRARY`, or
`ORBVI_HOST_SDK_SOURCE_DIR` when developing against a different SDK build.

Public C++ option/result structures are part of the Host SDK ABI. Headers and
libraries must therefore be refreshed as one bundle. Never copy a newer
`liborbvi_sdk.so` over a Bridge binary compiled with older headers; clean and
rebuild the Bridge after every bundle refresh. The host-SDK link test resolves
the rig-depth panorama symbol to reject a stale binary bundle.

`libturbojpeg` is statically linked into the bundled `liborbvi_sdk.so`, so ROS
bridge users do not need to install TurboJPEG separately.

GCC 9 is required for rebuilding or replacing the Host SDK binary package. The
ROS bridge package can be built with the compiler provided by its ROS build
environment.

When refreshing this bundle, verify the release binaries before committing:

```bash
file libs/linux-x86_64/liborbvi_sdk.so.0.1.0
file libs/linux-aarch64/liborbvi_sdk.so.0.1.0
readelf -d libs/linux-x86_64/liborbvi_sdk.so.0.1.0 | grep NEEDED
```

`readelf` should list OpenSSL, pthread, libstdc++, libc and the dynamic loader,
but not `libturbojpeg`.
