#pragma once

#include <algorithm>
#include <cctype>
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

}  // namespace orbvi_ros_bridge
