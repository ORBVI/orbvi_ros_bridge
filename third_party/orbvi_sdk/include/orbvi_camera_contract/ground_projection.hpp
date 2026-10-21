#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <locale>
#include <sstream>
#include <string>

namespace orbvi_camera_contract {

// T_parent_device maps device FLU coordinates into the robot parent frame.
// Device origin is the front optical camera centre, as in SDK ROS_BODY.
// Plane: n.x*x + n.y*y + n.z*z + d = 0 in parent_frame; metres, xyzw.
struct RobotGroundProjection {
  std::string parent_frame = "base_link";
  std::string child_frame = "orbvi_link";
  std::array<double, 3> xyz{};
  std::array<double, 4> quaternion{0, 0, 0, 1};
  std::array<double, 4> ground_plane{0, 0, 1, 0};
};

inline bool ValidProjectionFrame(const std::string& frame) {
  if (frame.empty() || frame.size() > 63 || frame.front() == '/' || frame.back() == '/') return false;
  for (unsigned char c : frame)
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '/')) return false;
  return frame.find("//") == std::string::npos;
}

inline bool ValidRobotGroundProjection(const RobotGroundProjection& p) {
  if (!ValidProjectionFrame(p.parent_frame) || !ValidProjectionFrame(p.child_frame) ||
      p.parent_frame == p.child_frame) return false;
  for (double v : p.xyz) if (!std::isfinite(v)) return false;
  double q2 = 0;
  for (double v : p.quaternion) { if (!std::isfinite(v)) return false; q2 += v*v; }
  if (!std::isfinite(q2) || std::abs(q2-1) > 1e-9) return false;
  for (double v : p.ground_plane) if (!std::isfinite(v)) return false;
  const auto& n = p.ground_plane;
  const double length = std::hypot(std::hypot(n[0],n[1]),n[2]);
  return std::isfinite(length) && length > 1e-9 && std::abs(n[2])/length >= 0.1;
}

inline std::array<double, 4> ProjectionQuaternionFromRpy(const std::array<double, 3>& rpy) {
  const double cr=std::cos(rpy[0]/2), sr=std::sin(rpy[0]/2);
  const double cp=std::cos(rpy[1]/2), sp=std::sin(rpy[1]/2);
  const double cy=std::cos(rpy[2]/2), sy=std::sin(rpy[2]/2);
  return {sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
          cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy};
}

// Bounded, versioned metadata; classic locale and round-trip doubles.
inline std::string SerializeRobotGroundProjection(const RobotGroundProjection& p) {
  if (!ValidRobotGroundProjection(p)) return {};
  std::ostringstream s; s.imbue(std::locale::classic());
  s.precision(std::numeric_limits<double>::max_digits10);
  s << "v1 " << p.parent_frame << ' ' << p.child_frame;
  for (double v : p.xyz) s << ' ' << v;
  for (double v : p.quaternion) s << ' ' << v;
  for (double v : p.ground_plane) s << ' ' << v;
  return s.str();
}

inline bool ParseRobotGroundProjection(const std::string& text, RobotGroundProjection* out) {
  if (!out || text.size() >= 512) return false;
  std::istringstream s(text); s.imbue(std::locale::classic());
  std::string version; RobotGroundProjection p;
  if (!(s >> version >> p.parent_frame >> p.child_frame) || version != "v1") return false;
  for (double& v : p.xyz) if (!(s >> v)) return false;
  for (double& v : p.quaternion) if (!(s >> v)) return false;
  for (double& v : p.ground_plane) if (!(s >> v)) return false;
  s >> std::ws;
  if (!s.eof() || !ValidRobotGroundProjection(p)) return false;
  *out = p; return true;
}

}  // namespace orbvi_camera_contract
