#pragma once

#include <string>

namespace orbvi_sdk {

enum class PeerContract { Current, Legacy0daaf911 };
enum class CompatibilityMode { Auto, Current, Legacy0daaf911 };

struct PeerCompatibility {
  PeerContract contract = PeerContract::Current;
  std::string selection_source;
  std::string app_version;
  // Running session identity; provisioning changes DeviceInfo::device_sn first.
  std::string device_sn;
};

inline constexpr bool IsValidPeerContract(PeerContract contract) {
  switch (contract) {
    case PeerContract::Current:
    case PeerContract::Legacy0daaf911:
      return true;
  }
  return false;
}

inline constexpr bool IsValidCompatibilityMode(CompatibilityMode mode) {
  switch (mode) {
    case CompatibilityMode::Auto:
    case CompatibilityMode::Current:
    case CompatibilityMode::Legacy0daaf911:
      return true;
  }
  return false;
}

inline const char* ToString(PeerContract contract) {
  switch (contract) {
    case PeerContract::Current:
      return "current-v1";
    case PeerContract::Legacy0daaf911:
      return "legacy-0daaf911";
  }
  return "unknown";
}

}  // namespace orbvi_sdk
