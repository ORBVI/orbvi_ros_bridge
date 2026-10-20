#pragma once
#include "orbvi_sdk/capture.hpp"

namespace orbvi_sdk {
// Linux local-process consumer transport. One independent stream group/socket;
// it never starts, stops, reconnects or changes the source owner.
struct CaptureIpcOptions {
  std::size_t max_clients = 8;
  // Fixed whole-packet I/O deadline, 100..10000ms. A connected Prepared
  // client may wait indefinitely before Start and occupies one client slot.
  std::uint32_t io_timeout_ms = 1000;
  std::size_t max_packet_bytes = 16U * 1024U * 1024U;
  CaptureConsumerOptions consumer{CaptureDelivery::Ordered,
                                  {8, 16U * 1024U * 1024U, 2000}};
};
class CaptureIpcServer {
public:
  CaptureIpcServer();
  CaptureIpcServer(CaptureIpcServer &&) noexcept;
  CaptureIpcServer &operator=(CaptureIpcServer &&) noexcept;
  ~CaptureIpcServer();
  void stop(SubscriptionStopMode mode = SubscriptionStopMode::DrainAccepted);
  CaptureIpcServer(const CaptureIpcServer &) = delete;
  CaptureIpcServer &operator=(const CaptureIpcServer &) = delete;
  // Upstream is Ordered; preview Latest belongs to the receiving consumer.
  // Refuses an existing path. Parent directory must already exist and be
  // private.
  static Result<CaptureIpcServer> Open(CaptureOutlet outlet,
                                       const std::string &socket_path,
                                       CaptureIpcOptions options = {});

private:
  struct Impl;
  explicit CaptureIpcServer(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};
struct CaptureIpcFinalReport {
  bool ended = false, cancelled = false, failed = false;
  std::string exporter_id, reason;
  std::vector<CaptureConsumerReport> streams;
};
namespace detail {
struct CaptureIpcMonitorState;
}
class CaptureIpcMonitor {
public:
  CaptureIpcMonitor() = default;
  explicit CaptureIpcMonitor(
      std::shared_ptr<detail::CaptureIpcMonitorState> state)
      : state_(std::move(state)) {}
  CaptureIpcFinalReport report() const;
  // Empty seals all streams. Explicit IDs seal only this connection's streams.
  Result<void> seal(std::vector<StreamId> streams = {}) const;

private:
  std::shared_ptr<detail::CaptureIpcMonitorState> state_;
};
struct CaptureIpcConnection {
  std::unique_ptr<CaptureSource> source;
  CaptureIpcMonitor monitor;
};
// After Start, waiting for the next packet uses max(1000ms, io_timeout_ms).
// Once its first byte arrives, the entire packet must fit io_timeout_ms.
Result<CaptureIpcConnection>
ConnectCaptureIpc(const std::string &socket_path, std::vector<StreamId> streams,
                  std::uint32_t io_timeout_ms = 1000);
// A disconnect is terminal. DrainAccepted requests only this connection's
// admitted interval and checks remote counters; it never proves sensor
// completeness.
Result<std::unique_ptr<CaptureSource>>
OpenIpcCaptureSource(const std::string &socket_path, StreamId stream,
                     std::uint32_t io_timeout_ms = 1000);
Result<std::unique_ptr<CaptureSource>>
OpenIpcCaptureSource(const std::string &socket_path,
                     std::vector<StreamId> streams,
                     std::uint32_t io_timeout_ms = 1000);
} // namespace orbvi_sdk
