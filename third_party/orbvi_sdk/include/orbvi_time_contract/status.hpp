#pragma once

#include "orbvi_camera_contract/product.hpp"
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace orbvi_time_contract {
enum class Support { Unknown, Unsupported, Supported };
enum class State { Unknown, Unsupported, Waiting, Locked, Lost };
struct Capabilities {
  Support mid360_ptp = Support::Unknown;
  Support external_clock = Support::Unknown;
};
inline constexpr Capabilities CapabilitiesFor(orbvi_camera_contract::Product product) {
  using P = orbvi_camera_contract::Product;
  if (product == P::Unknown) return {};
  return {product == P::FourVL ? Support::Supported : Support::Unsupported,
          product == P::FourVLite || product == P::FourP ? Support::Supported : Support::Unsupported};
}
inline constexpr const char* ToString(State state) {
  switch (state) {
    case State::Unsupported: return "UNSUPPORTED";
    case State::Waiting: return "WAITING";
    case State::Locked: return "LOCKED";
    case State::Lost: return "LOST";
    default: return "UNKNOWN";
  }
}
inline std::optional<State> ParseState(std::string_view text) {
  for (auto state : {State::Unknown, State::Unsupported, State::Waiting, State::Locked, State::Lost})
    if (text == ToString(state)) return state;
  return std::nullopt;
}
struct Channel {
  Support support = Support::Unknown;
  State state = State::Unknown;
  std::string source;
  std::string reason = "NOT_OBSERVED";
  // Age of the evidence, not a measured synchronization error. Missing is unknown.
  std::optional<std::uint64_t> age_ms;
};
struct Status { Channel mid360_ptp; Channel external_clock; };
inline Status ForProduct(orbvi_camera_contract::Product product) {
  const auto caps = CapabilitiesFor(product);
  Status out;
  out.mid360_ptp.support = caps.mid360_ptp;
  out.external_clock.support = caps.external_clock;
  for (auto* channel : {&out.mid360_ptp, &out.external_clock})
    if (channel->support == Support::Unsupported) {
      channel->state = State::Unsupported;
      channel->reason = "PRODUCT_UNSUPPORTED";
    }
  if (caps.mid360_ptp == Support::Supported) out.mid360_ptp.source = "mid360_ptp";
  if (caps.external_clock == Support::Supported) out.external_clock.source = "pps_nmea";
  return out;
}
inline void WriteMetadata(const Status& status, std::map<std::string, std::string>& metadata) {
  metadata["time_status_version"] = "2";
  const auto put = [&](const char* prefix, const Channel& value) {
    const std::string key(prefix);
    metadata[key + "_state"] = ToString(value.state);
    metadata[key + "_source"] = value.source;
    metadata[key + "_reason"] = value.reason;
    metadata.erase(key + "_age_ms");
    if (value.age_ms) metadata[key + "_age_ms"] = std::to_string(*value.age_ms);
  };
  put("mid360_ptp", status.mid360_ptp);
  put("external_clock", status.external_clock);
}
}  // namespace orbvi_time_contract
