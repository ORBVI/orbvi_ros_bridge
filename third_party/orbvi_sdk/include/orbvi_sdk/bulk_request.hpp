#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace orbvi_sdk {

// Host-side mirror of the device Bulk control-plane framing (Chapter 5).
// The wire format MUST match src/orbvi_sdk_core/include/orbvi_sdk_core/bulk_request.hpp
// byte-for-byte.

constexpr std::uint32_t kBulkControlMagic = 0x4f425243u;
constexpr std::uint16_t kBulkControlVersion = 1;

enum class BulkControlType : std::uint8_t {
  Request = 1,
  Response = 2,
};

struct BulkControlMessage {
  BulkControlType type = BulkControlType::Request;
  std::uint32_t request_id = 0;
  std::string route;
  std::uint16_t status = 0;
  std::string body;
};

std::vector<std::uint8_t> EncodeBulkControl(const BulkControlMessage& msg);

enum class BulkControlDecodeStatus {
  Ok,
  NeedMoreData,
  InvalidMagic,
  UnsupportedVersion,
  Truncated,
};

const char* ToString(BulkControlDecodeStatus status);

struct BulkControlDecodeResult {
  BulkControlDecodeStatus status = BulkControlDecodeStatus::NeedMoreData;
  BulkControlMessage message;
  std::size_t bytes_consumed = 0;
  std::string error;

  bool ok() const { return status == BulkControlDecodeStatus::Ok; }
};

BulkControlDecodeResult DecodeBulkControl(const std::uint8_t* data,
                                          std::size_t size);
BulkControlDecodeResult DecodeBulkControl(const std::vector<std::uint8_t>& data);

}  // namespace orbvi_sdk
