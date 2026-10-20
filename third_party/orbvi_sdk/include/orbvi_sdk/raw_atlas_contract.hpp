#pragma once

#include "orbvi_camera_contract/rig.hpp"
#include <array>
#include <cstdint>

namespace orbvi_sdk {

// Host-side mirror of the device RAW_ATLAS_MJPEG spatial layout contract.
// See the device-side header (orbvi_sdk_core/raw_atlas_contract.hpp) for the
// authoritative documentation; the constants and tile mapping here MUST stay
// byte-for-byte identical. Both copies are guarded by cross-side tests.

constexpr std::uint32_t kRawAtlasTileWidth = 1280;
constexpr std::uint32_t kRawAtlasTileHeight = 1088;
constexpr std::uint32_t kRawAtlasTilesPerRow = 2;
constexpr std::uint32_t kRawAtlasTileCount = 4;

constexpr std::uint32_t kRawAtlasWidth = kRawAtlasTileWidth * kRawAtlasTilesPerRow;
constexpr std::uint32_t kRawAtlasHeight = kRawAtlasTileHeight * kRawAtlasTilesPerRow;

constexpr const char* kRawAtlasLayout = "grid_2x2_1280x1088";
// Direct MIPI ports, with no mounting-direction assumption.
constexpr const char* kFourPRawAtlasLayout = "grid_2x2_1280x1088_ports_v1";
constexpr std::array<std::uint32_t, 4> kFourPRawAtlasCameraIds = {0, 1, 2, 3};

enum class RawAtlasTileIndex : std::uint8_t {
  Front = 0,
  Right = 1,
  Rear = 2,
  Left = 3,
};

constexpr std::array<std::uint32_t, kRawAtlasTileCount> kRawAtlasCameraIds = {
    orbvi_camera_contract::kX5Rig.camera_ids[0],
    orbvi_camera_contract::kX5Rig.camera_ids[1],
    orbvi_camera_contract::kX5Rig.camera_ids[2],
    orbvi_camera_contract::kX5Rig.camera_ids[3],
};

constexpr const char* const kRawAtlasTileDirections[kRawAtlasTileCount] = {
    "front",
    "right",
    "rear",
    "left",
};

inline std::uint32_t RawAtlasCameraIdForTile(RawAtlasTileIndex tile) {
  return kRawAtlasCameraIds[static_cast<std::size_t>(tile)];
}

// 4V Lite only. 4P tile indices identify connectors, not physical directions.
inline const char* RawAtlasDirectionForTile(RawAtlasTileIndex tile) {
  return kRawAtlasTileDirections[static_cast<std::size_t>(tile)];
}

inline std::uint32_t RawAtlasTileRow(RawAtlasTileIndex tile) {
  return static_cast<std::uint32_t>(tile) / kRawAtlasTilesPerRow;
}

inline std::uint32_t RawAtlasTileCol(RawAtlasTileIndex tile) {
  return static_cast<std::uint32_t>(tile) % kRawAtlasTilesPerRow;
}

inline std::uint32_t RawAtlasTileOffsetX(RawAtlasTileIndex tile) {
  return RawAtlasTileCol(tile) * kRawAtlasTileWidth;
}

inline std::uint32_t RawAtlasTileOffsetY(RawAtlasTileIndex tile) {
  return RawAtlasTileRow(tile) * kRawAtlasTileHeight;
}

}  // namespace orbvi_sdk
