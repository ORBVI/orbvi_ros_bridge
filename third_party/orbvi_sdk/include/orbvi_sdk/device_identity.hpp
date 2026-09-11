#pragma once

#include <cstdint>
#include <string>

#include "orbvi_sdk/status.hpp"

namespace orbvi_sdk {

// Legacy identity snapshot and offline consistency helper (Chapter 8).
//
// This type does not select or enable a mixed runtime transport profile. The
// production Client supports only NetworkNetwork (no USB resources) and
// UvcVendorBulk (all USB functions selected by one exact VID/PID/iSerial).
// The helper remains useful for diagnostics and persisted identity records:
//
//   - USB iSerial is matched against the Network device SN to bind the physical
//     device. Standard UVC metadata does NOT carry an ORBVI boot_id, so USB
//     alone cannot provide boot_id.
//   - boot_id is obtained from the protocol-plane FrameSync (Network or Bulk)
//     as the session epoch. The host stores it once and clears its PTS match
//     state when it changes (device reboot). boot_id is NOT required from the
//     USB side; only the protocol side provides it.
//   - device_model and protocol_version are secondary cross-checks.
//
// A mismatch means the two snapshots cannot describe the same device; callers
// must treat it as DeviceIdentityMismatch.
struct DeviceIdentity {
  // USB descriptor iSerialNumber (empty if not connected via USB).
  std::string usb_serial;
  // Device SN reported over the network control plane (/v1/device/info).
  std::string network_sn;
  // Per-boot identifier from FrameSync (0 if not yet observed).
  std::uint64_t boot_id = 0;
  std::string device_model;
  std::string protocol_version;
};

struct DeviceIdentityCheck {
  bool match = true;
  SdkErrorCode code = SdkErrorCode::Ok;
  std::string reason;
};

// Compare two device identity snapshots. This is FAIL-CLOSED: if any
// comparable field differs, or if critical fields are missing entirely, the
// check fails with DeviceIdentityMismatch. It is not a transport routing API.
//
// When comparing legacy USB-side and Network-side snapshots, the USB serial
// and network SN are compared symmetrically: the "serial" of each identity
// (usb_serial if present, else network_sn) must match the other's. This
// diagnostic comparison does not make that runtime combination supported.
inline DeviceIdentityCheck CheckIdentityMatch(const DeviceIdentity& a,
                                              const DeviceIdentity& b) {
  DeviceIdentityCheck result;

  // First, verify internal consistency: if an identity carries BOTH a USB
  // serial and a network SN, they must agree. A conflict means the object is
  // self-contradictory → fail-closed.
  auto check_internal = [](const DeviceIdentity& d,
                           DeviceIdentityCheck& r) -> bool {
    if (!d.usb_serial.empty() && !d.network_sn.empty() &&
        d.usb_serial != d.network_sn) {
      r.match = false;
      r.code = SdkErrorCode::DeviceIdentityMismatch;
      r.reason = "usb_serial and network_sn disagree within the same identity";
      return false;
    }
    return true;
  };
  if (!check_internal(a, result)) return result;
  if (!check_internal(b, result)) return result;

  // Helper: the "effective serial" of an identity — usb_serial if present,
  // else network_sn. Both have been verified consistent above.
  auto effective_serial = [](const DeviceIdentity& d) -> const std::string& {
    return d.usb_serial.empty() ? d.network_sn : d.usb_serial;
  };

  const std::string& sa = effective_serial(a);
  const std::string& sb = effective_serial(b);

  // Serial: both sides MUST have a non-empty serial to pass. Missing serial
  // means we cannot verify identity → fail-closed.
  if (sa.empty() || sb.empty()) {
    result.match = false;
    result.code = SdkErrorCode::DeviceIdentityMismatch;
    result.reason = "device serial missing on one or both links";
    return result;
  }
  if (sa != sb) {
    result.match = false;
    result.code = SdkErrorCode::DeviceIdentityMismatch;
    result.reason = "device serial differs across links";
    return result;
  }

  // boot_id: only compared when BOTH sides provide a non-zero value. The USB
  // side (UVC metadata) does not carry boot_id, so it is typically 0 on the
  // USB identity. The protocol side (Network/Bulk FrameSync) provides it. A
  // mismatch (both non-zero but different) means a device reboot happened
  // mid-session → fail. A missing value (one side is 0) does NOT fail, because
  // the physical device is already bound by serial above.
  if (a.boot_id != 0 && b.boot_id != 0 && a.boot_id != b.boot_id) {
    result.match = false;
    result.code = SdkErrorCode::DeviceIdentityMismatch;
    result.reason = "boot_id differs across links";
    return result;
  }

  // device_model and protocol_version: only checked if both sides provide a
  // value; a mismatch fails, but a missing value does not (these are secondary
  // checks).
  if (!a.device_model.empty() && !b.device_model.empty() &&
      a.device_model != b.device_model) {
    result.match = false;
    result.code = SdkErrorCode::DeviceIdentityMismatch;
    result.reason = "device model differs across links";
    return result;
  }

  if (!a.protocol_version.empty() && !b.protocol_version.empty() &&
      a.protocol_version != b.protocol_version) {
    result.match = false;
    result.code = SdkErrorCode::DeviceIdentityMismatch;
    result.reason = "protocol version differs across links";
    return result;
  }

  return result;
}

}  // namespace orbvi_sdk
