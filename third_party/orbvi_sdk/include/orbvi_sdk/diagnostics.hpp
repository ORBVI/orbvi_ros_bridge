#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "orbvi_sdk/frame.hpp"

namespace orbvi_sdk {

enum class ReadinessState {
  Unknown,
  Ready,
  Blocked,
  Unsupported,
  Degraded,
  DebugFallback,
};

enum class ReleaseEvidence {
  Unknown,
  RealContinuousStream,
  RecordingReplay,
  SampleEndpointFallback,
  DebugSynthetic,
};

struct ResourceBudget {
  std::uint32_t sample_window_sec = 5;
  double sdk_core_cpu_avg_limit_percent = 150.0;
  double sdk_core_cpu_peak_10s_limit_percent = 250.0;
  std::uint64_t sdk_core_rss_limit_bytes = 512ull * 1024ull * 1024ull;
  std::size_t fd_limit = 256;
  std::size_t worker_thread_limit = 16;
  std::size_t queue_depth_limit = 256;
};

struct ResourceSummary {
  std::uint32_t sample_window_sec = 5;
  double sdk_core_cpu_avg_percent = 0.0;
  double sdk_core_cpu_peak_10s_percent = 0.0;
  std::uint64_t sdk_core_rss_bytes = 0;
  std::size_t fd_count = 0;
  std::size_t worker_thread_count = 0;
  std::size_t receive_queue_depth = 0;
  std::size_t decode_queue_depth = 0;
  bool within_budget = false;
  std::string blocked_reason;
};

struct DiagnosticSummary {
  ReadinessState readiness = ReadinessState::Unknown;
  ReleaseEvidence evidence = ReleaseEvidence::Unknown;
  TransportMode transport_mode = TransportMode::SampleEndpoint;
  ResourceSummary resources;
  std::string sdk_version;
  std::string device_serial;
  std::string capability_summary;
  std::string calibration_status;
  std::string blocked_reason;
  std::vector<std::string> warnings;
};

inline const char* ToString(ReadinessState state) {
  switch (state) {
    case ReadinessState::Ready:
      return "READY";
    case ReadinessState::Blocked:
      return "BLOCKED";
    case ReadinessState::Unsupported:
      return "UNSUPPORTED";
    case ReadinessState::Degraded:
      return "DEGRADED";
    case ReadinessState::DebugFallback:
      return "DEBUG_FALLBACK";
    case ReadinessState::Unknown:
    default:
      return "UNKNOWN";
  }
}

inline const char* ToString(ReleaseEvidence evidence) {
  switch (evidence) {
    case ReleaseEvidence::RealContinuousStream:
      return "real_continuous_stream";
    case ReleaseEvidence::RecordingReplay:
      return "recording_replay";
    case ReleaseEvidence::SampleEndpointFallback:
      return "sample_endpoint_fallback";
    case ReleaseEvidence::DebugSynthetic:
      return "debug_synthetic";
    case ReleaseEvidence::Unknown:
    default:
      return "unknown";
  }
}

inline ResourceBudget DefaultV09ResourceBudget() {
  return {};
}

inline bool IsReleaseReadyState(ReadinessState state) {
  return state == ReadinessState::Ready;
}

inline bool IsReleaseReadyEvidence(ReleaseEvidence evidence) {
  return evidence == ReleaseEvidence::RealContinuousStream;
}

inline bool IsReleaseReadyTransport(TransportMode mode) {
  return mode == TransportMode::StreamTransport;
}

inline bool IsReleaseReadyDiagnostic(const DiagnosticSummary& summary) {
  return IsReleaseReadyState(summary.readiness) &&
         IsReleaseReadyEvidence(summary.evidence) &&
         IsReleaseReadyTransport(summary.transport_mode) &&
         summary.resources.within_budget &&
         summary.blocked_reason.empty();
}

}  // namespace orbvi_sdk
