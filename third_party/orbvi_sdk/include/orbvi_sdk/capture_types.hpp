#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "orbvi_sdk/frame.hpp"
#include "orbvi_sdk/subscription.hpp"

namespace orbvi_sdk {

enum class CaptureState { Prepared, Running, Ended, Stopping, Stopped, Failed };
enum class CaptureDelivery { Ordered, Latest };
enum class CaptureEventKind { Started, ConsumerJoined, ConsumerStopped, Gap, SourceFailed, SourceEnded, Stopped, ConsumerFailed, ConsumerDropped };
enum class CaptureEventScope { Source, Consumer, Session };

// Failure category is independent of scope and of the optional human text.
enum class CaptureFailureCode { None, SourceGap, SourceFailure, ConsumerBudget,
  ConsumerAge, CallbackException, ReplayBudget };
inline const char* ToString(CaptureFailureCode code) {
  switch (code) {
    case CaptureFailureCode::None: return "none";
    case CaptureFailureCode::SourceGap: return "source_gap";
    case CaptureFailureCode::SourceFailure: return "source_failure";
    case CaptureFailureCode::ConsumerBudget: return "consumer_budget";
    case CaptureFailureCode::ConsumerAge: return "consumer_age";
    case CaptureFailureCode::CallbackException: return "callback_exception";
    case CaptureFailureCode::ReplayBudget: return "replay_budget";
  }
  return "unknown";
}
struct CaptureSequenceGap {
  bool known = false;
  std::uint64_t boot_id = 0, expected = 0, observed = 0, missing = 0;
  bool source_reported = false, delivery_known = false;
  std::uint64_t delivery_session_id = 0, delivery_cursor = 0;
};
struct CaptureStreamHealth {
  std::uint64_t received = 0, first_received_monotonic_ns = 0, last_received_monotonic_ns = 0;
  std::uint64_t last_frame_timestamp_ns = 0;
  // Computed on report(), using Host steady clock; never subtract device time.
  std::uint64_t last_frame_age_ms = 0;
};

struct CaptureCalibration {
  std::string version, hash, status, raw_json;
};

struct CaptureContext {
  std::vector<StreamId> streams;
  MetadataMap snapshots;
  bool replay = false;
};
struct CaptureSourceEvent {
  CaptureEventKind kind = CaptureEventKind::SourceFailed;
  StreamId stream = StreamId::Unknown;
  std::string reason;
};
struct CaptureSourceReport {
  std::map<StreamId, SubscriptionReport> streams;
  std::string error;
};

struct CaptureOptions {
  // Covers shared logical-frame allocations, including application-held leases.
  // Transport/reassembly queues have their own explicit source budgets.
  std::size_t max_shared_bytes = 256U * 1024U * 1024U;
  std::size_t max_frame_bytes = 16U * 1024U * 1024U;
  std::size_t max_consumers = 16;
  std::size_t max_events = 128;
};
struct CaptureTimingOptions {
  // Device-envelope temporal coverage when the manifest contains images + IMU.
  // The recorder separately checks every image view and the source proof.
  std::uint32_t imu_preroll_ms = 100;
  std::uint32_t startup_timeout_ms = 15000;
  std::uint32_t imu_tail_timeout_ms = 2000;
};
struct CaptureReplayOptions {
  // Shared pending deliveries, charged conservatively once per consumer.
  // They use the session pool in addition to retained consumer reservations.
  std::size_t max_pending_frames = 256;
  std::size_t max_pending_bytes = 32U * 1024U * 1024U;
  std::uint32_t stall_timeout_ms = 2000;
};
struct CaptureConsumerOptions {
  // Ordered fails on known source discontinuity within its active interval.
  // Latest continues; the source Gap event remains observable on the outlet.
  CaptureDelivery delivery = CaptureDelivery::Ordered;
  // Live: queued + executing + application-retained leases. Replay Ordered
  // lookahead uses CaptureReplayOptions until it obtains a consumer lease.
  // Latest evicts queued leases only, never application-held memory.
  OrderedDeliveryOptions budget;
};
struct CaptureOrigin {
  std::string owner_id;
  std::uint64_t epoch = 0, ordinal = 0;
};
struct CapturedFrame {
  std::shared_ptr<const OwnedFrame> frame;
  std::uint64_t session_epoch = 0;
  std::uint64_t ordinal = 0;  // Host ingress ordinal, never a sensor sequence.
};
using CaptureCallback = std::function<void(const CapturedFrame&)>;
struct CaptureConsumerRequest {
  StreamId stream = StreamId::Unknown;
  CaptureConsumerOptions options;
  CaptureCallback callback;
};

struct CaptureEvent {
  std::uint64_t revision = 0;
  std::uint64_t session_epoch = 0;
  CaptureEventKind kind = CaptureEventKind::Started;
  CaptureEventScope scope = CaptureEventScope::Session;
  StreamId stream = StreamId::Unknown;
  std::uint64_t consumer_id = 0;
  std::uint64_t ordinal = 0;
  std::string reason;
  CaptureFailureCode failure = CaptureFailureCode::None;
  CaptureSequenceGap sequence_gap;
  std::uint64_t observed_monotonic_ns = 0;
};
struct CaptureEvents {
  std::vector<CaptureEvent> events;
  std::uint64_t lost_events = 0;
  std::uint64_t latest_revision = 0;
};
struct CaptureConsumerReport {
  std::uint64_t consumer_id = 0;
  StreamId stream = StreamId::Unknown;
  bool accepting = false;
  bool stopped = false;
  bool cancelled = false;
  bool failed = false;
  std::uint64_t admitted = 0;
  std::uint64_t completed = 0;
  std::uint64_t dropped = 0;
  std::uint64_t first_ordinal = 0;
  std::uint64_t last_ordinal = 0;
  std::size_t held_bytes = 0;
  std::size_t held_frames = 0;
  std::string error;
  CaptureFailureCode failure = CaptureFailureCode::None;
  std::size_t peak_held_bytes = 0, peak_held_frames = 0;
};
struct CaptureSequenceReport {
  bool identity_observed = false;
  std::uint64_t boot_id = 0, last_sequence = 0;
  std::uint64_t observed_frames = 0, unverified_frames = 0;
  std::uint64_t gap_events = 0, missing_frames = 0;
  // First known discontinuity, retained even when the bounded event log rolls.
  // No gaps is not proof of the source start/end boundaries or unverified input.
  std::string error;
};
struct CaptureReport {
  CaptureState state = CaptureState::Prepared;
  bool cancelled = false;
  std::uint64_t session_epoch = 0;
  std::map<StreamId, std::uint64_t> received;
  std::size_t shared_bytes = 0;
  std::size_t peak_shared_bytes = 0;
  std::size_t reserved_bytes = 0;
  CaptureSourceReport source;
  std::string error;
  // Latched totals survive destruction of completed subscription handles.
  std::uint64_t consumers_failed = 0, source_gap_events = 0;
  std::string first_consumer_error;
  // Sum of stopped consumers, not a unique count of source frames. Live
  // consumers expose their current counters through subscription.report().
  std::uint64_t stopped_consumers = 0, stopped_consumer_admitted = 0;
  std::uint64_t stopped_consumer_completed = 0, stopped_consumer_dropped = 0;
  std::map<StreamId, CaptureStreamHealth> stream_health;
};


}  // namespace orbvi_sdk
