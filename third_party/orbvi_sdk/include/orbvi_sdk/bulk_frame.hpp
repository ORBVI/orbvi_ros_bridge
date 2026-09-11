#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace orbvi_sdk {

// Host-side mirror of the device Bulk data-plane chunking (Chapter 3).
// The wire format MUST match src/orbvi_sdk_core/include/orbvi_sdk_core/bulk_frame.hpp
// byte-for-byte.

constexpr std::uint32_t kBulkChunkMagic = 0x4f42434bu;
constexpr std::uint16_t kBulkChunkVersion = 1;

constexpr std::uint16_t kBulkChunkFlagFirst = 1u << 0;
constexpr std::uint16_t kBulkChunkFlagLast = 1u << 1;

constexpr std::uint32_t kBulkDefaultChunkSize = 16 * 1024;
constexpr std::uint32_t kBulkMaxPayloadSize = 65u * 1024u * 1024u;
constexpr std::size_t kBulkChunkHeaderSize = 24;



struct BulkChunkHeader {
  std::uint32_t magic = kBulkChunkMagic;
  std::uint16_t version = kBulkChunkVersion;
  std::uint16_t flags = 0;
  std::uint32_t transfer_id = 0;
  std::uint32_t offset = 0;
  std::uint32_t total_payload_size = 0;
  std::uint32_t chunk_payload_size = 0;
};

std::vector<std::vector<std::uint8_t>> EncodeBulkTransfer(
    std::uint32_t transfer_id,
    const std::uint8_t* payload,
    std::size_t payload_size,
    std::uint32_t chunk_size = kBulkDefaultChunkSize);

enum class BulkReassembleStatus {
  Ok,
  NeedMoreData,
  InvalidMagic,
  UnsupportedVersion,
  OffsetMismatch,
  SizeMismatch,
  CrcMismatch,
};

const char* ToString(BulkReassembleStatus status);

class BulkReassembler {
 public:
  explicit BulkReassembler(std::uint32_t expected_transfer_id = 0);
  BulkReassembleStatus Feed(const std::uint8_t* data, std::size_t size);
  bool complete() const { return complete_; }
  const std::vector<std::uint8_t>& payload() const { return buffer_; }
  std::uint32_t transfer_id() const { return transfer_id_; }
  void Reset(std::uint32_t expected_transfer_id);

 private:
  std::uint32_t expected_transfer_id_ = 0;
  std::uint32_t transfer_id_ = 0;
  std::uint32_t total_payload_size_ = 0;
  std::uint32_t next_offset_ = 0;
  bool complete_ = false;
  std::vector<std::uint8_t> buffer_;
};

}  // namespace orbvi_sdk
