#pragma once

#include "orbvi_time_contract/status.hpp"

namespace orbvi_time_contract {

// Evidence supplied by the device runtime. No UART, PPS, ntpq or worker owns
// this interface; SDK consumers only read an observation and its capture epoch.
struct ExternalClockSnapshot {
  bool pps_locked = false;
  bool nmea_time_valid = false;
  bool time_synced = false;
  bool previously_locked = false;
  std::string blocked_reason = "X5_UTC_NOT_OBSERVED";
  std::int64_t snapshot_realtime_ns = 0;
  std::int64_t locked_since_monotonic_ns = 0;
  std::int64_t locked_since_realtime_ns = 0;
  std::int64_t observed_monotonic_ns = 0;
  std::int64_t realtime_monotonic_offset_ns = 0;
};

// Snapshot must return a bounded, fresh observation without device I/O.
class ExternalClockProvider {
 public:
  virtual ~ExternalClockProvider() = default;
  virtual ExternalClockSnapshot Snapshot() const = 0;
};

inline Channel ExternalClockStatus(const ExternalClockSnapshot& status,
                                   std::int64_t now_monotonic_ns) {
  Channel out;
  out.support = Support::Supported;
  out.source = "pps_nmea";
  out.state = status.time_synced ? State::Locked : status.previously_locked ? State::Lost :
      status.observed_monotonic_ns > 0 ? State::Waiting : State::Unknown;
  out.reason = status.blocked_reason;
  if (status.observed_monotonic_ns > 0 && now_monotonic_ns >= status.observed_monotonic_ns)
    out.age_ms = static_cast<std::uint64_t>(now_monotonic_ns - status.observed_monotonic_ns) / 1000000;
  return out;
}
}  // namespace orbvi_time_contract
