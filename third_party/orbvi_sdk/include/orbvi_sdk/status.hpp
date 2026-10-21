#pragma once

#include <cstdint>
#include <string>
#include <utility>

namespace orbvi_sdk {

enum class StreamId : std::uint16_t {
  Unknown = 0,
  RawFisheye = 1,
  RectifiedFisheye = 2,
  Imu = 3,
  LidarPointCloud = 4,
  LidarImu = 5,
  Disparity = 6,
  VioPose = 7,
  PanoDisplay = 8,
  BevDisplay = 9,
  Bev = BevDisplay,
  FrameSync = 10,
  RegisteredCloud = 11,
};

enum class FrameFormat : std::uint16_t {
  Unknown = 0,
  FisheyeMjpeg = 1,
  RectifiedMjpeg = 2,
  // Wire compatibility name. Payload semantics are one complete rectified JPEG per frame.
  RectifiedMjpegBundle = RectifiedMjpeg,
  ImuSample = 3,
  Mid360Custom = 4,
  LidarImuSample = 5,
  Disparity32F = 6,
  Disparity16UQ8 = 7,
  VioPose = 8,
  PanoDisplayJpeg = 9,
  BevDisplayJpeg = 10,
  BevMjpeg = BevDisplayJpeg,
  RawAtlasMjpeg = 11,
  FrameSyncV2 = 12,
  RegisteredPointXyziRgb = 13,
  Reserved = 65535,
};

enum class Compression : std::uint16_t {
  None = 0,
  Jpeg = 1,
  Mjpeg = 2,
  Reserved = 65535,
};

enum class TimestampDomain : std::uint8_t {
  Unknown = 0,
  Monotonic = 1,
  Realtime = 2,
  Sensor = 3,
};

enum class SdkErrorCode {
  Ok = 0,
  ConnectionFailed,
  Timeout,
  ProtocolError,
  CrcMismatch,
  MetadataMismatch,
  UnsupportedStream,
  StreamBlocked,
  CalibrationMissing,
  DecodeFailed,
  ResourceLimit,
  NotConnected,
  ImageTransportUnavailable,
  ProtocolTransportUnavailable,
  UsbSuperspeedRequired,
  BandwidthExhausted,
  FrameSyncUnavailable,
  DeviceReenumerating,
  DeviceIdentityMismatch,
  RecordingIoError,
  RecordingIncomplete,
  RecordingCapabilityUnavailable,
  CalibrationInvalidInput,
  CalibrationFailed,
  CalibrationIoError,
};

struct SdkError {
  SdkErrorCode code = SdkErrorCode::Ok;
  std::string message;

  explicit operator bool() const { return code != SdkErrorCode::Ok; }
};

template <typename T>
class Result {
 public:
  Result(T value) : value_(std::move(value)) {}
  Result(SdkError error) : error_(std::move(error)) {}

  bool ok() const { return error_.code == SdkErrorCode::Ok; }
  explicit operator bool() const { return ok(); }
  const T& value() const { return value_; }
  T& value() { return value_; }
  const SdkError& error() const { return error_; }

 private:
  T value_{};
  SdkError error_{};
};

template <>
class Result<void> {
 public:
  Result() = default;
  Result(SdkError error) : error_(std::move(error)) {}

  bool ok() const { return error_.code == SdkErrorCode::Ok; }
  explicit operator bool() const { return ok(); }
  const SdkError& error() const { return error_; }

 private:
  SdkError error_{};
};

const char* ToString(StreamId stream_id);
const char* ToString(FrameFormat format);
const char* ToString(Compression compression);
const char* ToString(TimestampDomain timestamp_domain);
const char* ToString(SdkErrorCode code);

bool ParseStreamId(const std::string& name, StreamId* out);
bool ParseFrameFormat(const std::string& name, FrameFormat* out);
bool ParseCompression(const std::string& name, Compression* out);
bool ParseTimestampDomain(const std::string& name, TimestampDomain* out);
FrameFormat ExpectedFormatForStream(StreamId stream_id);
bool IsAllowedFormatForStream(StreamId stream_id, FrameFormat format);
bool IsImageStream(StreamId stream_id);
bool IsImageFormat(FrameFormat format);

}  // namespace orbvi_sdk
