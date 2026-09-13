#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>

// Header-only shared contract: no SDK transport, OpenCV, ROS or hardware types.
namespace orbvi_camera_contract {

enum class Platform { Unknown, Rk3588, X5 };
enum class Role { Unknown = -1, Front = 0, Right = 1, Rear = 2, Left = 3 };
inline constexpr const char* kContractVersion = "orbvi-adjacent-stereo-v1";
inline constexpr std::array<Role, 4> kRoles{{Role::Front, Role::Right, Role::Rear, Role::Left}};

inline constexpr const char* ToString(Role role) {
  switch (role) {
    case Role::Front: return "front";
    case Role::Right: return "right";
    case Role::Rear: return "rear";
    case Role::Left: return "left";
    default: return "unknown";
  }
}
inline constexpr const char* ToString(Platform platform) {
  switch (platform) {
    case Platform::Rk3588: return "rk3588";
    case Platform::X5: return "x5";
    default: return "unknown";
  }
}

struct StereoPair {
  std::size_t pair_id;
  Role direction;
  std::size_t left_index;
  std::size_t right_index;
  int left_camera_id;
  int right_camera_id;
};

struct Rig {
  // Physical calibration and raw-atlas role order. These are NOT pair IDs.
  std::array<int, 4> camera_ids;

  constexpr std::optional<StereoPair> pair(std::size_t index) const {
    if (index >= kRoles.size()) return std::nullopt;
    const auto right = (index + 1) % kRoles.size();
    return StereoPair{index, kRoles[index], index, right,
                      camera_ids[index], camera_ids[right]};
  }
  constexpr std::optional<StereoPair> stereo(Role direction) const {
    for (std::size_t i = 0; i < kRoles.size(); ++i)
      if (kRoles[i] == direction) return pair(i);
    return std::nullopt;
  }
};

// The only platform ID tables. Never substitute MIPI indices or pair IDs here.
inline constexpr Rig kRk3588Rig{{{3, 2, 4, 5}}};
inline constexpr Rig kX5Rig{{{0, 3, 2, 1}}};
inline constexpr std::optional<Rig> ForPlatform(Platform platform) {
  switch (platform) {
    case Platform::Rk3588: return kRk3588Rig;
    case Platform::X5: return kX5Rig;
    default: return std::nullopt;
  }
}

inline constexpr unsigned CameraMask(const Rig& rig) {
  unsigned mask = 0;
  for (const auto id : rig.camera_ids) {
    if (id < 0 || id >= 32) return 0;
    mask |= 1U << id;
  }
  return mask;
}
inline std::string CameraIdsCsv(const Rig& rig) {
  std::string text;
  for (const auto id : rig.camera_ids) {
    if (!text.empty()) text += ',';
    text += std::to_string(id);
  }
  return text;
}
}  // namespace orbvi_camera_contract
