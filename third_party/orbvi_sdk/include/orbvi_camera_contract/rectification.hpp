#pragma once
#include <array>
#include <cmath>
#include <optional>

namespace orbvi_camera_contract {
struct StereoGeometry {
  // Columns are the rectified x/y/z axes expressed in the reference frame.
  std::array<double, 9> rotation_reference_from_rectified{};
  double baseline_m = 0;
  double fx_px = 0;
  double fy_px = 0;
  double cx_px = 0;
  double cy_px = 0;
};

// Preserve the established horizontal surround-rig rectification convention.
// Left/right order determines +x and the outward +z; never silently swap them.
inline std::optional<StereoGeometry> Rectification(
    const std::array<double, 3>& left_m,
    const std::array<double, 3>& right_m, int width, int height) {
  if (width <= 0 || height <= 0 || width % 2 || height % 2 ||
      static_cast<long long>(width) * 256 != static_cast<long long>(height) * 320)
    return std::nullopt;
  std::array<double, 3> x{}, y{}, z{};
  double norm2 = 0;
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(left_m[i]) || !std::isfinite(right_m[i])) return std::nullopt;
    x[i] = right_m[i] - left_m[i]; norm2 += x[i] * x[i];
  }
  const double baseline = std::sqrt(norm2);
  if (!std::isfinite(baseline) || baseline <= 1e-9) return std::nullopt;
  for (auto& value : x) value /= baseline;
  z = {{-x[2], 0, x[0]}};
  const double zn = std::sqrt(z[0]*z[0] + z[2]*z[2]);
  if (zn <= 1e-9) return std::nullopt;
  for (auto& value : z) value /= zn;
  y = {{z[1]*x[2]-z[2]*x[1], z[2]*x[0]-z[0]*x[2], z[0]*x[1]-z[1]*x[0]}};
  const double yn = std::sqrt(y[0]*y[0]+y[1]*y[1]+y[2]*y[2]);
  StereoGeometry result;
  for (std::size_t row = 0; row < 3; ++row) {
    result.rotation_reference_from_rectified[row*3] = x[row];
    result.rotation_reference_from_rectified[row*3+1] = y[row]/yn;
    result.rotation_reference_from_rectified[row*3+2] = z[row];
  }
  result.baseline_m = baseline;
  result.fx_px = result.fy_px = 158.0 * static_cast<double>(width)/320.0;
  result.cx_px = width/2.0; result.cy_px = height/2.0;
  return result;
}
}  // namespace orbvi_camera_contract
