#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "orbvi_sdk/status.hpp"

namespace orbvi_sdk {

using MetadataMap = std::map<std::string, std::string>;

struct FrameHeaderV1 {
  std::uint16_t version = 1;
  std::uint16_t header_size = 40;
  std::uint16_t flags = 0;
  StreamId stream_id = StreamId::Unknown;
  FrameFormat format = FrameFormat::Unknown;
  Compression compression = Compression::None;
  std::uint16_t frame_id_size = 0;
  std::uint64_t timestamp_ns = 0;
  std::uint32_t metadata_size = 0;
  std::uint32_t payload_size = 0;
  std::uint32_t crc32 = 0;
};

enum class TransportMode {
  StreamTransport,
  SampleEndpoint,
};

struct SubscriptionStats {
  TransportMode transport_mode = TransportMode::SampleEndpoint;
  double observed_rate_hz = 0.0;
  std::size_t payload_size_bytes = 0;
  double delivery_latency_ms = 0.0;
  double decode_latency_ms = 0.0;
  double max_decode_latency_ms = 0.0;
  std::size_t receive_queue_depth = 0;
  std::size_t decode_queue_depth = 0;
  std::uint64_t delivered_frames = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t skipped_decodes = 0;
  std::uint64_t decode_failures = 0;
  std::uint64_t parse_errors = 0;
  std::uint64_t reconnect_count = 0;
  std::uint64_t calibration_cache_hits = 0;
  std::uint64_t calibration_cache_misses = 0;
  std::size_t calibration_cache_entries = 0;
  std::uint64_t overflow_events = 0;
  std::uint64_t backpressure_events = 0;
  double cpu_percent_sample = 0.0;
  std::uint64_t rss_bytes_sample = 0;
  std::string overflow_action;
  std::string blocked_reason;
};

struct FrameView {
  StreamId stream_id = StreamId::Unknown;
  FrameFormat format = FrameFormat::Unknown;
  Compression compression = Compression::None;
  std::uint64_t timestamp_ns = 0;
  std::string frame_id;
  const MetadataMap* metadata = nullptr;
  const std::uint8_t* payload_data = nullptr;
  std::size_t payload_size = 0;
  std::string calibration_version;
  std::string calibration_hash;
};

struct OwnedFrame {
  FrameHeaderV1 header;
  std::string frame_id;
  MetadataMap metadata;
  std::vector<std::uint8_t> payload;

  FrameView view() const;
};

std::uint32_t Crc32(const std::uint8_t* data, std::size_t size);
std::uint32_t FrameCrc32(const std::uint8_t* data, std::size_t size);
std::string SerializeMetadata(const MetadataMap& metadata);
Result<MetadataMap> ParseMetadata(const std::string& serialized);
Result<OwnedFrame> ParseFrameBytes(const std::uint8_t* data, std::size_t size);
Result<OwnedFrame> ParseFrameBytes(const std::vector<std::uint8_t>& bytes);

}  // namespace orbvi_sdk
