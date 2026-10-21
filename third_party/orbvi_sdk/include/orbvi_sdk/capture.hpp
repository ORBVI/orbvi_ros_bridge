#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "orbvi_sdk/capture_types.hpp"
#include "orbvi_sdk/client_options.hpp"
#include "orbvi_sdk/device_contract.hpp"

namespace orbvi_sdk {

// A source owns transport/replay scheduling. It moves each logical frame once.
// stop must quiesce callbacks before returning. A false sink result requests
// cancellation; producers must never silently restart into another epoch.
// stop must also be safe before start and after failed/partial start. A source
// reports Gap/SourceFailed/SourceEnded independently of its frame callback.
class CaptureSource {
 public:
  using FrameSink = std::function<bool(OwnedFrame)>;
  using EventSink = std::function<void(CaptureSourceEvent)>;
  virtual ~CaptureSource() = default;
  virtual CaptureContext context() const = 0;
  virtual Result<void> start(FrameSink frames, EventSink events) = 0;
  virtual CaptureSourceReport stop(SubscriptionStopMode mode) = 0;
};

namespace detail { struct CaptureConsumer; struct CaptureCore; }
class CaptureSubscription {
 public:
  CaptureSubscription() = default;
  CaptureSubscription(CaptureSubscription&&) noexcept;
  CaptureSubscription& operator=(CaptureSubscription&&) noexcept;
  ~CaptureSubscription();
  CaptureSubscription(const CaptureSubscription&) = delete;
  CaptureSubscription& operator=(const CaptureSubscription&) = delete;
  CaptureConsumerReport report() const;
  // Close ingress without waiting for accepted callbacks. Use before stop()
  // when other streams must keep running while this stream drains.
  void seal();
  CaptureConsumerReport stop(SubscriptionStopMode mode = SubscriptionStopMode::DrainAccepted);
 private:
  friend class CaptureOutlet;
  explicit CaptureSubscription(std::shared_ptr<detail::CaptureConsumer> state);
  std::shared_ptr<detail::CaptureConsumer> state_;
};

// A copyable, non-owning consumer capability. It cannot start/stop the source.
// The owner must outlive use; expired outlets reject subscriptions.
class CaptureImageConsumer;
struct CaptureImageConsumerOptions;
struct CaptureImagePoolOptions;
struct CaptureImageResourceReport;
class CaptureOutlet {
 public:
  CaptureOutlet() = default;
  Result<CaptureSubscription> subscribe(StreamId stream, CaptureConsumerOptions options,
      CaptureCallback callback) const;
  // All streams become visible at one Host ingress boundary, or none do.
  // Valid before start and while running; does not provide historical preroll.
  Result<std::vector<CaptureSubscription>> subscribeGroup(
      std::vector<CaptureConsumerRequest> requests) const;
  // On-demand RAW atlas tiles shared among overlapping consumers of this epoch.
  Result<std::vector<std::shared_ptr<const CompressedImage>>> rawAtlasImages(
      const CapturedFrame& frame, std::uint8_t requested_mask = 0xf) const;
  Result<std::shared_ptr<CaptureImageConsumer>> imageConsumer(CaptureImageConsumerOptions options) const;
  CaptureImageResourceReport imageResources() const;
  std::string originId() const;
  Result<CaptureOrigin> origin(const CapturedFrame& frame) const;
  CaptureContext context() const;
  Result<DeviceContract> contract() const;
  Result<std::shared_ptr<const CaptureCalibration>> calibration(
      const std::string& version, const std::string& hash) const;
  CaptureReport report() const;
  // Source sequence evidence, separate from Host ingress and consumer counters.
  std::map<StreamId, CaptureSequenceReport> continuity() const;
  CaptureEvents events(std::uint64_t after = 0,
      std::chrono::milliseconds wait = std::chrono::milliseconds(0)) const;
 private:
  friend class CaptureSession;
  explicit CaptureOutlet(std::weak_ptr<detail::CaptureCore> core) : core_(std::move(core)) {}
  std::weak_ptr<detail::CaptureCore> core_;
};

// The owner continues acquisition with zero consumers. A consumer cannot close
// the source. Attach sinks before start for a common Host start boundary.
class CaptureSession {
 public:
  CaptureSession() = default;
  CaptureSession(CaptureSession&&) noexcept;
  CaptureSession& operator=(CaptureSession&&) noexcept;
  ~CaptureSession();
  CaptureSession(const CaptureSession&) = delete;
  CaptureSession& operator=(const CaptureSession&) = delete;
  static Result<CaptureSession> Open(std::unique_ptr<CaptureSource> source,
                                     CaptureOptions options = {});
  static Result<CaptureSession> Open(std::unique_ptr<CaptureSource> source,
      CaptureOptions options, CaptureReplayOptions replay);
  static Result<CaptureSession> Open(std::unique_ptr<CaptureSource> source,
      CaptureOptions options, CaptureReplayOptions replay, const CaptureImagePoolOptions& images);
  static Result<CaptureSession> OpenDevice(ClientOptions client,
      std::vector<StreamId> streams = {StreamId::Imu, StreamId::RawFisheye},
      CaptureOptions options = {}, OrderedDeliveryOptions source_queue = {},
      CaptureTimingOptions timing = {});
  static Result<CaptureSession> OpenDevice(ClientOptions client, std::vector<StreamId> streams,
      CaptureOptions options, OrderedDeliveryOptions source_queue, CaptureTimingOptions timing,
      const CaptureImagePoolOptions& images);
  // Overrides replace the whole source budget for that manifest stream,
  // including a FrameSync input added by the device contract. Others use
  // source_queue. Invalid or unnegotiable budgets fail without clipping.
  static Result<CaptureSession> OpenDevice(ClientOptions client, std::vector<StreamId> streams,
      CaptureOptions options, OrderedDeliveryOptions source_queue, CaptureTimingOptions timing,
      const CaptureImagePoolOptions& images,
      const std::map<StreamId, OrderedDeliveryOptions>& source_queue_overrides);
  Result<CaptureSubscription> subscribe(StreamId stream, CaptureConsumerOptions options,
                                         CaptureCallback callback);
  CaptureOutlet outlet() const;
  Result<void> start();
  CaptureReport stop(SubscriptionStopMode mode = SubscriptionStopMode::DrainAccepted);
  CaptureContext context() const;
  Result<DeviceContract> contract() const;
  Result<std::shared_ptr<const CaptureCalibration>> calibration(
      const std::string& version, const std::string& hash) const;
  CaptureReport report() const;
  // Source sequence evidence, separate from Host ingress and consumer counters.
  std::map<StreamId, CaptureSequenceReport> continuity() const;
  CaptureEvents events(std::uint64_t after = 0,
      std::chrono::milliseconds wait = std::chrono::milliseconds(0)) const;
 private:
  explicit CaptureSession(std::shared_ptr<detail::CaptureCore> core);
  std::shared_ptr<detail::CaptureCore> core_;
};

}  // namespace orbvi_sdk
