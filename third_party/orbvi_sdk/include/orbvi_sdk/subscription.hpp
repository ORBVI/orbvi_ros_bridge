#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace orbvi_sdk {

// Per-consumer FIFO budget, independent of the shared transport budget.
// Raw frames only; overflow/age expiry stops this subscription visibly.
// Bounds queued frames, not a callback's working memory or retained copies.
struct OrderedDeliveryOptions {
  std::size_t max_frames = 64;
  std::size_t max_bytes = 64U * 1024U * 1024U;
  std::uint32_t max_age_ms = 2000;
};

enum class SubscriptionStopMode { Cancel, DrainAccepted };

// Evidence for one explicitly negotiated SDK Server export interval. This
// says nothing about samples lost before that export or a shared hardware
// boundary across streams. Old peers, USB and Host-only consumers are Unverified.
enum class DeliveryIntervalStatus { Unverified, Open, Complete, Incomplete, Cancelled };
inline const char* ToString(DeliveryIntervalStatus status) {
  switch (status) {
    case DeliveryIntervalStatus::Unverified: return "UNVERIFIED";
    case DeliveryIntervalStatus::Open: return "OPEN";
    case DeliveryIntervalStatus::Complete: return "COMPLETE";
    case DeliveryIntervalStatus::Incomplete: return "INCOMPLETE";
    case DeliveryIntervalStatus::Cancelled: return "CANCELLED";
  }
  return "UNVERIFIED";
}
struct DeliveryIntervalReport {
  DeliveryIntervalStatus status = DeliveryIntervalStatus::Unverified;
  std::string contract, scope;
  std::uint64_t session_id = 0;
  // Half-open cursor interval (start_cursor, end_cursor]. Empty is allowed.
  std::uint64_t start_cursor = 0, end_cursor = 0, received_cursor = 0;
  std::uint64_t published_frames = 0, received_frames = 0, gap_frames = 0;
  bool sealed = false;
};

// A queryable Host-side snapshot. stopped means stop/join has completed;
// failed/error can be observed before stop, without waiting for another frame.
// host_drained certifies only admitted callbacks. Always inspect error too:
// device stop can fail even if all Host callbacks completed. These counts do
// not authenticate device source boundaries or certify capture completeness.
struct SubscriptionReport {
  bool stopped = false;
  bool ordered = false;
  bool failed = false;
  bool cancelled = false;
  bool host_drained = false;
  std::uint64_t admitted = 0;
  std::uint64_t callbacks_completed = 0;
  // NOT_OBSERVED / EOF / DEADLINE / ERROR
  std::string transport_end = "NOT_OBSERVED";
  // NOT_TRACKED / DRAINED / INCOMPLETE
  std::string shared_drain = "NOT_TRACKED";
  // NONE / BULK_ROUTED_FRAMES / UVC_DEQUEUED_VIDEO
  std::string shared_scope = "NONE";
  std::uint64_t shared_checkpoint = 0;
  std::string error;
  DeliveryIntervalReport interval;
};

}  // namespace orbvi_sdk
