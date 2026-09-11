#pragma once

#include "orbvi_sdk/fisheye_rig.hpp"
#include "orbvi_sdk/status.hpp"

namespace orbvi_sdk {

struct DepthTileCalibration;

struct Pixel2d {
  double x = 0.0;
  double y = 0.0;
};
// Optical camera axes: +X image right, +Y image down, +Z forward.
struct Point3d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};
struct Ray3d {
  Point3d origin_m;
  Point3d direction;  // Unit length, in the same frame as origin_m.
};
struct Projection {
  Pixel2d pixel;
  bool inside_image = false;
};

enum class CameraProjectionModel { TripleSphere, RectifiedPinhole };
struct CameraModel {
  CameraProjectionModel model = CameraProjectionModel::TripleSphere;
  // Reuses the neutral rig contract; b/c are affine off-diagonal terms.
  // For RectifiedPinhole, xi/lambda/alpha must all be zero.
  PanoramaCameraIntrinsics intrinsics;
};

// Models are snapshots. Use intrinsics for the exact image resolution/crop;
// no implicit resizing, atlas offset, device I/O, or calibration refresh.
// Require positive dimensions/focals, 0 <= alpha < 1, xi/lambda > -1, and
// a nonsingular, positive-determinant affine intrinsic matrix.
Result<CameraModel> MakeCameraModel(const PanoramaCameraIntrinsics& intrinsics);
// Coordinates are local to this rectified tile, not to the full atlas.
Result<CameraModel> MakeRectifiedCameraModel(const DepthTileCalibration& tile);

// Integer coordinates denote pixel centres. Inside: [0,width-1] x [0,height-1].
// Out-of-image pixels are allowed when mathematically valid. TS uses the
// positive-root inverse branch and rejects ambiguous/noninvertible directions.
Result<Projection> Project(const CameraModel& camera,
                           const Point3d& point_camera);
Result<Point3d> Unproject(const CameraModel& camera, const Pixel2d& pixel);
Result<Point3d> UnprojectWithRange(const CameraModel& camera,
                                   const Pixel2d& pixel, double range_m);
// Positive optical Z depth; rejects rays on/behind the optical horizon.
Result<Point3d> UnprojectWithDepthZ(const CameraModel& camera,
                                    const Pixel2d& pixel, double depth_z_m);

// Raw TS rig only. Lookup is by camera_id, never vector position or role.
// Require complete rigid extrinsics; p_reference = R * p_camera + t (metres).
Result<Projection> ProjectFromReference(const PanoramaCalibration& rig,
                                        std::uint32_t camera_id,
                                        const Point3d& point_reference_m);
Result<Ray3d> UnprojectToReference(const PanoramaCalibration& rig,
                                   std::uint32_t camera_id,
                                   const Pixel2d& pixel);

}  // namespace orbvi_sdk
