#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace orbvi_sdk {

// Host-side mirror of the device OTA upload wire contract.
constexpr std::array<std::uint8_t, 6> kOtaUploadMagic = {
    'O', 'T', 'A', 'F', '0', '1'};
constexpr std::uint16_t kOtaUploadProtocolVersion = 1;
constexpr std::uint16_t kOtaUploadHeaderSize = 28;
constexpr std::uint16_t kOtaUploadFrameTypeChunk = 1;
constexpr std::size_t kOtaUploadCrcOffset = 22;
constexpr std::uint32_t kMaxOtaUploadMetadataBytes = 64u * 1024u;
constexpr std::uint32_t kMaxOtaUploadPayloadBytes = 64u * 1024u * 1024u;

constexpr const char* kNetworkOtaUploadTransport = "tcp_frame_stream";
constexpr const char* kBulkOtaUploadTransport = "vendor_bulk_otaf01";

// Data Bulk interface name; consumed by the BulkReader data binding.
constexpr const char* kBulkInterfaceName = "ORBVI Vendor Bulk";

// OTA Bulk interface name. Host discovers OTA endpoints at runtime from the
// exact OTA iInterface, vendor class, bulk type, and endpoint direction;
// descriptor placeholder addresses are not Host ABI.
constexpr const char* kBulkOtaInterfaceName = "ORBVI Vendor Bulk OTA";

constexpr void WriteOtaU16(
    std::array<std::uint8_t, kOtaUploadHeaderSize>* header,
    std::size_t offset,
    std::uint16_t value) {
  (*header)[offset] = static_cast<std::uint8_t>(value & 0xffu);
  (*header)[offset + 1] =
      static_cast<std::uint8_t>((value >> 8u) & 0xffu);
}

constexpr void WriteOtaU32(
    std::array<std::uint8_t, kOtaUploadHeaderSize>* header,
    std::size_t offset,
    std::uint32_t value) {
  for (std::size_t i = 0; i < 4; ++i) {
    (*header)[offset + i] =
        static_cast<std::uint8_t>((value >> (8u * i)) & 0xffu);
  }
}

constexpr std::array<std::uint8_t, kOtaUploadHeaderSize>
MakeOtaUploadFrameHeader(
    std::uint32_t metadata_size,
    std::uint32_t payload_size,
    std::uint32_t crc32) {
  std::array<std::uint8_t, kOtaUploadHeaderSize> header{};
  for (std::size_t i = 0; i < kOtaUploadMagic.size(); ++i) {
    header[i] = kOtaUploadMagic[i];
  }
  WriteOtaU16(&header, 6, kOtaUploadProtocolVersion);
  WriteOtaU16(&header, 8, kOtaUploadHeaderSize);
  WriteOtaU16(&header, 10, kOtaUploadFrameTypeChunk);
  WriteOtaU16(&header, 12, 0);
  WriteOtaU32(&header, 14, metadata_size);
  WriteOtaU32(&header, 18, payload_size);
  WriteOtaU32(&header, kOtaUploadCrcOffset, crc32);
  WriteOtaU16(&header, 26, 0);
  return header;
}

static_assert(MakeOtaUploadFrameHeader(0, 0, 0).size() == 28,
              "OTAF01 header size changed");

}  // namespace orbvi_sdk
