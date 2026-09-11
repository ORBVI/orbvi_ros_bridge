#pragma once

#include <cstdint>
#include <string>

namespace orbvi_sdk {

// Transport selection model for the ORBVI device SDK.
//
// Three explicit product profiles are supported:
//
//   profile        | image | non-image data | control | OTA
//   ---------------+-------+----------------+---------+-----
//   NetworkNetwork | Net   | Net            | Net     | Net
//   UvcOnly         | UVC   | -              | -       | -
//   UvcVendorBulk  | UVC   | Bulk           | Bulk    | Bulk
//
// Mixed combinations (for example UVC images over a Network control plane,
// or Network images with a Bulk control plane) are intentionally unsupported
// and cannot be represented in DeviceOptions. The default keeps runtime
// behavior identical for callers that rely on the network-only defaults;
// callers that configured the removed independent fields must migrate to one
// of these explicit profiles.
enum class TransportProfile : std::uint8_t {
  NetworkNetwork = 0,
  UvcVendorBulk = 1,
  UvcOnly = 2,
};

struct DeviceOptions {
  TransportProfile transport_profile = TransportProfile::NetworkNetwork;
};

inline const char* ToString(TransportProfile profile) {
  switch (profile) {
    case TransportProfile::NetworkNetwork:
      return "network-network";
    case TransportProfile::UvcVendorBulk:
      return "uvc-vendor-bulk";
    case TransportProfile::UvcOnly:
      return "uvc-only";
  }
  return "unknown";
}

inline bool ParseTransportProfile(const std::string& name,
                                  TransportProfile* out) {
  if (name == "network-network") {
    if (out != nullptr) {
      *out = TransportProfile::NetworkNetwork;
    }
    return true;
  }
  if (name == "uvc-vendor-bulk") {
    if (out != nullptr) {
      *out = TransportProfile::UvcVendorBulk;
    }
    return true;
  }
  if (name == "uvc-only") {
    if (out != nullptr) {
      *out = TransportProfile::UvcOnly;
    }
    return true;
  }
  return false;
}

// Result of validating a DeviceOptions. The only legal value is one of the
// TransportProfile enumerators; anything else is rejected explicitly
// instead of being interpreted.
struct DeviceOptionsValidation {
  bool ok = true;
  std::string reason;
};

inline DeviceOptionsValidation ValidateDeviceOptions(
    const DeviceOptions& options) {
  DeviceOptionsValidation result;
  if (options.transport_profile != TransportProfile::NetworkNetwork &&
      options.transport_profile != TransportProfile::UvcVendorBulk &&
      options.transport_profile != TransportProfile::UvcOnly) {
    result.ok = false;
    result.reason = "unknown transport profile";
    return result;
  }
  return result;
}

}  // namespace orbvi_sdk
