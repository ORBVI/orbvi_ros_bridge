#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "orbvi_sdk/client.hpp"

namespace orbvi_sdk {

struct RecordingOptions {
  std::string path;
  std::vector<StreamId> streams;
  bool include_calibration = true;
  bool include_device_status = true;
  std::size_t queue_capacity = 64;
  // Recorder startup may precede cold algorithm producers. Keep retrying
  // device sessions within this Host-side deadline instead of inheriting the
  // shorter single-session first-payload SLA.
  std::uint32_t subscription_first_frame_timeout_ms = 15000;
};

struct RecordingProvenance {
  std::string producer = "orbvi_sdk";
  // Required for every SDK-capable recording. metadata must also contain
  // nonempty firmware_app_version and sdk_protocol_version values.
  std::string software_version;
  std::string calibration_version;
  std::string calibration_hash;
  std::string calibration_json;
  std::string device_info_json;
  std::string device_status_json;
  // sdk_peer_contract selects current-v1 (default) or legacy-0daaf911 for the
  // entire session. Every written frame must have that PeerContract. Recorder
  // sets this from Client::getCompatibility(); it is not ORBVI1 frame metadata.
  MetadataMap metadata;
};

// These observations overlap: a source-sequence hole may include an SDK drop.
// Never add them together to claim a unique end-to-end lost-frame count.
struct RecordingLossStats {
  std::uint64_t source_sequence_gaps = 0;
  std::uint64_t sdk_receive_dropped_frames = 0;
  std::uint64_t recorder_queue_dropped_frames = 0;
  std::uint64_t observed_frames = 0;
  std::uint64_t unsequenced_frames = 0;
  std::uint64_t sequence_resets = 0;
};

struct RecordingStreamStats {
  StreamId stream = StreamId::Unknown;
  std::uint64_t generation = 0; // Recording-local epoch; latest epoch only.
  std::string boot_id;
  RecordingLossStats losses;
};

struct RecordingStats {
  std::uint64_t frames_written = 0;
  std::uint64_t bytes_written = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t gap_records = 0;
  std::uint64_t queue_overflow_events = 0;
  std::uint64_t reconnect_events = 0;
  std::uint64_t write_errors = 0;
  bool active = false;
  bool complete = false;
  std::string last_error;
  RecordingLossStats losses;
  // Bounded by subscribed stream count. Prior epochs are persisted as events.
  std::vector<RecordingStreamStats> streams;
};

struct RecordingCapabilities {
  bool system_replay = false;
  bool module_replay = false;
  bool sdk_replay = false;
};

struct RecordingSessionInfo {
  std::string producer;
  std::string software_version;
  RecordingCapabilities capabilities;
  std::string calibration_version;
  std::string calibration_hash;
  std::string calibration_json;
  std::string device_info_json;
  std::string device_status_json;
  MetadataMap metadata;
  std::uint64_t frames_written = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t gap_records = 0;
  bool complete = false;
  std::string terminal_reason;
  bool loss_accounting_available = false;
  RecordingLossStats losses;
};

struct RecordedFrame {
  std::uint64_t recorded_timeline_ns = 0;
  std::uint64_t source_id = 0;
  OwnedFrame frame;
};

class RecordingWriter {
public:
  RecordingWriter();
  ~RecordingWriter();

  RecordingWriter(const RecordingWriter &) = delete;
  RecordingWriter &operator=(const RecordingWriter &) = delete;
  RecordingWriter(RecordingWriter &&) noexcept;
  RecordingWriter &operator=(RecordingWriter &&) noexcept;

  static Result<RecordingWriter> Open(const RecordingOptions &options,
                                      const RecordingProvenance &provenance);

  Result<void> writeFrame(const OwnedFrame &frame,
                          std::uint64_t recorded_timeline_ns = 0,
                          std::uint64_t source_id = 0);
  Result<void> recordGap(std::uint64_t first_source_id,
                         std::uint64_t last_source_id,
                         std::uint64_t dropped_count, const std::string &reason,
                         std::uint64_t recorded_timeline_ns = 0);
  Result<void> recordEvent(const std::string &name, const std::string &details,
                           std::uint64_t recorded_timeline_ns = 0);
  // A nonempty failure reason writes an incomplete footer (e.g. source adapter
  // failed before all queued data/evidence could be persisted).
  Result<RecordingStats> stop(const std::string &failure_reason = {});
  RecordingStats stats() const;
  bool active() const;

private:
  struct Impl;
  explicit RecordingWriter(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

class RecordingReader {
public:
  RecordingReader();
  ~RecordingReader();

  RecordingReader(const RecordingReader &) = delete;
  RecordingReader &operator=(const RecordingReader &) = delete;
  RecordingReader(RecordingReader &&) noexcept;
  RecordingReader &operator=(RecordingReader &&) noexcept;

  // Restores sdk_peer_contract from session metadata. Older recordings without
  // the key use Current; empty, unknown or duplicate declarations are rejected.
  static Result<RecordingReader> Open(const std::string &path);

  const RecordingSessionInfo &sessionInfo() const;
  Result<std::optional<RecordedFrame>> nextFrame();
  Result<void> rewind();

private:
  struct Impl;
  explicit RecordingReader(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

class RecordingSession {
public:
  RecordingSession();
  ~RecordingSession();

  RecordingSession(const RecordingSession &) = delete;
  RecordingSession &operator=(const RecordingSession &) = delete;
  RecordingSession(RecordingSession &&) noexcept;
  RecordingSession &operator=(RecordingSession &&) noexcept;

  Result<RecordingStats> stop();
  RecordingStats stats() const;
  bool active() const;

private:
  friend class Recorder;
  struct Impl;
  explicit RecordingSession(std::shared_ptr<Impl> impl);
  std::shared_ptr<Impl> impl_;
};

class Recorder {
public:
  static Result<RecordingSession> Start(Client &client,
                                        const RecordingOptions &options);
};

using ReplayFrameCallback = std::function<void(const RecordedFrame &)>;

class ReplaySource {
public:
  ReplaySource();
  ~ReplaySource();

  ReplaySource(const ReplaySource &) = delete;
  ReplaySource &operator=(const ReplaySource &) = delete;
  ReplaySource(ReplaySource &&) noexcept;
  ReplaySource &operator=(ReplaySource &&) noexcept;

  static Result<ReplaySource> Open(const std::string &path);

  const RecordingSessionInfo &sessionInfo() const;
  Result<bool> step(ReplayFrameCallback callback);
  Result<void> rewind();

private:
  struct Impl;
  explicit ReplaySource(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

} // namespace orbvi_sdk
