#pragma once

#include <cstdint>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "orbvi_sdk/bulk_frame.hpp"
#include "orbvi_sdk/bulk_request.hpp"
#include "orbvi_sdk/frame.hpp"
#include "orbvi_sdk/status.hpp"
#include "orbvi_sdk/uvc_contract.hpp"

namespace orbvi_sdk {

class BulkReceiveRouter;

namespace detail {
class BulkUsbRuntime;
class UsbBackend;
class BulkReaderTestPeer;
}

// Host-side Vendor Bulk reader (Chapter 3/5: Bulk + Network coexistence).
//
// The Bulk reader opens the Vendor Bulk endpoint, reads ORBVI1 Frame bytes
// chunked via BulkChunkHeader, reassembles them via BulkReassembler, and
// feeds the complete frame bytes into ParseFrameBytes — exactly the same
// path as the Network stream reader. The protocol semantics are identical;
// only the transport differs (USB Bulk vs TCP).
//
// Bulk also carries the control plane via BulkControlMessage frames. The
// reader transparently dispatches control responses to the same handler as
// HTTP — the caller does not see a difference.

// Bounds enforced by BulkReader::Open (via ValidateBulkReaderConfig) before any
// backend is created or opened. Out-of-range values are rejected with
// InvalidSelector.
inline constexpr std::uint32_t kBulkReaderMaxReadTimeoutMs = 10000;
inline constexpr std::uint32_t kBulkReaderMaxControlResponseAttempts = 1024;
inline constexpr std::uint32_t kBulkReaderMaxReadErrors = 1024;
// Total number of distinct stream slots retained by the shared-IN router.
// The device contract admits up to eight concurrently active logical streams;
// each slot contains at most one Latest frame, so this remains a fixed bound.
inline constexpr std::uint32_t kBulkReaderMaxQueuedDataFrames = 8;
inline constexpr std::uint32_t kBulkReaderMaxPendingControlResponses = 32;
// Synchronous libusb calls are sliced so Close() can observe cancellation and
// join an active reader within one short USB call instead of one full frame.
inline constexpr std::uint32_t kBulkReaderIoSliceMs = 100;

struct BulkReaderConfig {
  // USB composite identity (VID/PID/iSerial) of the device exposing the
  // Vendor Bulk interface.
  UsbCompositeSelector usb_selector;
  // Optional explicit Vendor Bulk binding. Leave the default all-zero/0xff
  // value to discover the data interface and its UDC-assigned endpoints from
  // the exact "ORBVI Vendor Bulk" iInterface at Open().
  VendorBulkBinding bulk_binding;
  // Read timeout per chunk in ms. Must be nonzero and <=
  // kBulkReaderMaxReadTimeoutMs.
  std::uint32_t read_timeout_ms = 2000;
  // Max consecutive read errors before giving up. Must be <=
  // kBulkReaderMaxReadErrors.
  std::uint32_t max_read_errors = 10;
  // ReadFrame caps the number of complete shared-IN frames it will route.
  // SendControlRequest uses this as a multiplier for its absolute response
  // deadline; frames belonging to another concurrent caller do not consume
  // that caller's logical attempt budget. Must be nonzero and <= the bound.
  std::uint32_t control_response_attempts = 32;
  // Shared-IN demux queues are bounded. This is the number of distinct stream
  // slots; each slot is always Latest(1), and ready slots are consumed fairly.
  // Control requests reserve bounded active slots before sending. Unsolicited,
  // timed-out and duplicate responses are rejected; a legal active response
  // is never evicted by another response.
  std::uint32_t max_queued_data_frames = kBulkReaderMaxQueuedDataFrames;
  std::uint32_t max_pending_control_responses = 32;
};

// Result of opening the Bulk endpoint.
enum class BulkReaderStatus {
  Ok,
  DeviceNotFound,
  OpenFailed,
  ClaimFailed,
  Unsupported,
  InvalidSelector,
  AmbiguousDevice,
  PermissionDenied,
  DescriptorMismatch,
  DeviceIdentityMismatch,
  CloseFailed,
};

const char* ToString(BulkReaderStatus status);

// Pure config validation used by BulkReader::Open(). Returns Ok only when every
// timing/attempt field is within the documented bounds; otherwise
// InvalidSelector. Safe to call without a USB backend.
BulkReaderStatus ValidateBulkReaderConfig(const BulkReaderConfig& config);

// Reads ORBVI1 frames from the Vendor Bulk endpoint. One instance per
// USB Bulk connection (not per stream — Bulk multiplexes streams in a
// single connection, unlike Network which uses one TCP connection per
// stream).
//
// Threading: ReadFrame() and SendControlRequest() may run concurrently. A
// single input owner lock covers each complete Bulk transfer and the router is
// independently protected, so competing callers never read or mutate the
// shared IN reassembler at the same time. Open()/Close() must not race with
// another Open()/Close().
class BulkReader {
 public:
  BulkReader();
  ~BulkReader();

  BulkReader(const BulkReader&) = delete;
  BulkReader& operator=(const BulkReader&) = delete;

  // Open the USB Bulk endpoint.
  BulkReaderStatus Open(const BulkReaderConfig& config);

  // Read one complete ORBVI1 frame (blocking with timeout).
  // Returns the frame bytes on success, or an error.
  Result<std::vector<std::uint8_t>> ReadFrame();

  // Send a Bulk control request and read the response.
  // request_id is allocated by the SDK so concurrent callers cannot reuse or
  // reorder caller-owned identifiers. IDs are never reused for the lifetime of
  // this BulkReader, including Close/Open. Exhaustion returns ResourceLimit;
  // transparent reconnect deliberately does not reset the counter.
  Result<BulkControlMessage> SendControlRequest(
      const std::string& route, const std::string& body_json);

  // Same request path with a caller-owned absolute deadline. Lock wait, OUT
  // transfer and shared-IN response routing all consume this one budget; the
  // operation never starts another USB slice after the deadline expires.
  Result<BulkControlMessage> SendControlRequestUntil(
      const std::string& route,
      const std::string& body_json,
      std::chrono::steady_clock::time_point deadline);

  bool IsOpen() const;

  // Performs bounded best-effort cleanup. Release/driver-reattach failures are
  // returned as CloseFailed after all remaining resources have been closed.
  BulkReaderStatus Close();

 private:
  friend class detail::BulkReaderTestPeer;

  BulkReaderConfig config_;
  std::unique_ptr<detail::UsbBackend> injected_backend_;
  std::unique_ptr<detail::BulkUsbRuntime> runtime_;
  BulkReassembler reassembler_;
  std::vector<std::uint8_t> input_staging_;
  std::unique_ptr<BulkReceiveRouter> router_;
  mutable std::mutex state_mutex_;
  std::timed_mutex input_mutex_;
  std::timed_mutex output_mutex_;
  std::mutex router_mutex_;
  // SDK-owned request ids give O(1) no-reuse state across Close/Open on the
  // same object. A libusb re-claim does not prove that stale FunctionFS IN
  // bytes were flushed, so reconnect never resets this counter.
  std::uint32_t next_request_id_ = 1;
  std::atomic<bool> closing_{false};

  // Read raw bytes from the Bulk IN endpoint.
  Result<std::vector<std::uint8_t>> ReadRawChunk(
      const std::chrono::steady_clock::time_point* deadline = nullptr);
  Result<std::vector<std::uint8_t>> ReadWireFrame(
      const std::chrono::steady_clock::time_point* deadline = nullptr);
  Result<BulkControlMessage> SendControlRequestImpl(
      const std::string& route,
      const std::string& body_json,
      const std::chrono::steady_clock::time_point* deadline);
};

}  // namespace orbvi_sdk
