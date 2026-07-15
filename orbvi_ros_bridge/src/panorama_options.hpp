#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>

#include "orbvi_sdk/panorama.hpp"

namespace orbvi_ros_bridge {

inline std::string NormalizePanoramaOptionToken(std::string value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  value = value.substr(begin, end - begin + 1);
  std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::replace(value.begin(), value.end(), '-', '_');
  return value;
}

inline bool ParsePanoramaSeamMode(
    const std::string& value,
    orbvi_sdk::PanoramaSeamMode* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string token = NormalizePanoramaOptionToken(value);
  if (token == "fixed" || token == "static") {
    *out = orbvi_sdk::PanoramaSeamMode::Fixed;
    return true;
  }
  if (token == "dynamic" || token == "dynamic_programming" || token == "dp") {
    *out = orbvi_sdk::PanoramaSeamMode::DynamicProgramming;
    return true;
  }
  return false;
}

inline bool ParsePanoramaBlendMode(
    const std::string& value,
    orbvi_sdk::PanoramaBlendMode* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string token = NormalizePanoramaOptionToken(value);
  if (token == "feather") {
    *out = orbvi_sdk::PanoramaBlendMode::Feather;
    return true;
  }
  if (token == "primary" || token == "primary_only" || token == "none") {
    *out = orbvi_sdk::PanoramaBlendMode::PrimaryOnly;
    return true;
  }
  if (token == "multiband" || token == "multi_band" || token == "mode2" ||
      token == "mode_2" || token == "2") {
    *out = orbvi_sdk::PanoramaBlendMode::MultiBand;
    return true;
  }
  return false;
}

inline std::uint32_t PanoramaOutputHeight(
    const orbvi_sdk::PanoramaStitchOptions& options) {
  const std::uint64_t crop =
      static_cast<std::uint64_t>(options.crop_top) + options.crop_bottom;
  return crop < options.height
             ? options.height - static_cast<std::uint32_t>(crop)
             : 0u;
}

inline bool SupportsDepthAssistedSeamRoi(
    const orbvi_sdk::PanoramaStitchOptions& options) {
  return options.blend_mode == orbvi_sdk::PanoramaBlendMode::MultiBand &&
         options.seam_mode == orbvi_sdk::PanoramaSeamMode::Fixed &&
         options.seam_blend_px > 0u &&
         !options.seam_ghost_suppression;
}

}  // namespace orbvi_ros_bridge
