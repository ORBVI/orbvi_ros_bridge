#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include "orbvi_sdk/image.hpp"
#include "orbvi_sdk/transport_options.hpp"
#include "orbvi_sdk/uvc_contract.hpp"

namespace orbvi_sdk {

struct ClientOptions {
  std::string host = "127.0.0.1";
  std::uint16_t control_port = 18088;
  std::uint32_t connect_timeout_ms = 2000;
  std::uint32_t read_timeout_ms = 2000;
  std::size_t max_receive_queue_depth = 32;
  DecodePolicy default_decode_policy;
  bool allow_sample_endpoint_fallback = false;
  std::uint32_t ota_preferred_chunk_bytes = 4u * 1024u * 1024u;
  bool ota_low_memory_upload = false;
  std::uint32_t ota_finalize_timeout_ms = 180000;
  // Dual-transport selection (Chapter 1). Defaults to Network for both, which
  // is fully backward-compatible with the existing network-only client. UVC
  // is an explicit Linux V4L2 transport; protocol Bulk and Bulk OTA use
  // distinct exact USB interfaces.
  DeviceOptions device_options;
  // Selects one USB composite parent. Child UVC functions are resolved by
  // iInterface. UvcVendorBulk requires paired video+metadata nodes; UvcOnly
  // binds the single video node without inventing FrameSync metadata.
  // Protocol Bulk requires exact nonzero VID/PID and a nonempty iSerial. The
  // SDK never falls back to Network when this identity is absent or rejected.
  UsbCompositeSelector usb_selector;
  // Backward-compatible selector hint. If set, this must identify the USB
  // composite sysfs parent. A single /dev/videoN child is ambiguous and is
  // rejected before any control-plane probe.
  std::string usb_device_path;
  // UvcOnly has no control identity: explicitly select the physical layout.
  // UvcVendorBulk discovers 4P from device info instead.
  std::string uvc_only_raw_layout = kRawAtlasLayout;
  // Preserve strict current parsing by default. Auto requires a recognized
  // declaration/build; unversioned legacy firmware needs explicit selection.
  CompatibilityMode compatibility_mode = CompatibilityMode::Current;
  // Optional application alias for diagnostics; endpoints and credentials are
  // never used as implicit log identity. At most 128 characters are emitted.
  std::string diagnostic_name;
};

struct SubscribeOptions {
  DecodePolicy decode_policy;
  std::size_t max_receive_queue_depth = 32;
  std::uint32_t first_frame_timeout_ms = 5000;
  bool require_streaming_transport = false;
  bool allow_sample_endpoint_fallback = false;
};


}  // namespace orbvi_sdk
