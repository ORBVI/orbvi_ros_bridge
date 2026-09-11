#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "orbvi_sdk/frame.hpp"

namespace orbvi_sdk {

constexpr std::size_t kLivoxPointCloudHeaderBytes = 16U;
constexpr std::size_t kLivoxPointBytes = 19U;
constexpr std::size_t kLivoxImuPayloadBytes = 37U * sizeof(double);

enum class LinearAccelerationUnit : std::uint8_t {
  Unknown = 0,
  StandardGravity,
  MetersPerSecondSquared,
};

struct LidarImuParameters {
  LinearAccelerationUnit linear_acceleration_unit =
      LinearAccelerationUnit::Unknown;
  double linear_acceleration_scale_to_mps2 = 0.0;
};

struct LivoxPoint {
  std::uint32_t offset_time_ns = 0;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::uint8_t reflectivity = 0;
  std::uint8_t tag = 0;
  std::uint8_t line = 0;
};

struct LivoxPointCloud {
  std::uint64_t timebase_ns = 0;
  std::uint8_t lidar_id = 0;
  std::array<std::uint8_t, 3> reserved{};
  std::vector<LivoxPoint> points;
};

struct LivoxImuSample {
  std::array<double, 4> orientation{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  std::array<double, 9> orientation_covariance{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 9> linear_acceleration_covariance{};
};

struct LivoxPointCloudFrame {
  std::uint32_t sequence = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  LivoxPointCloud cloud;
};

struct LivoxImuFrame {
  std::uint32_t sequence = 0;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  LivoxImuSample sample;
};

Result<LivoxPointCloud> DecodeLivoxPointCloudPayload(
    const std::uint8_t* data, std::size_t size);
Result<LivoxPointCloud> DecodeLivoxPointCloudPayload(
    const std::vector<std::uint8_t>& data);
Result<LivoxImuSample> DecodeLivoxImuPayload(
    const std::uint8_t* data, std::size_t size);
Result<LivoxImuSample> DecodeLivoxImuPayload(
    const std::vector<std::uint8_t>& data);
Result<LivoxPointCloudFrame> DecodeLivoxPointCloudFrame(
    const FrameView& frame);
Result<LivoxImuFrame> DecodeLivoxImuFrame(const FrameView& frame);
const char* ToString(LinearAccelerationUnit unit);
Result<LidarImuParameters> ParseLidarImuParametersJson(const std::string& json);

}  // namespace orbvi_sdk
