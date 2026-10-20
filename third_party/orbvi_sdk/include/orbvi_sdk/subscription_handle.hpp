#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <vector>
#include "orbvi_sdk/image.hpp"
#include "orbvi_sdk/subscription.hpp"
#include "orbvi_sdk/depth.hpp"
#include "orbvi_sdk/panorama.hpp"

namespace orbvi_sdk {

struct FrameDelivery {
  OwnedFrame frame;
  std::optional<OwnedDecodedImage> decoded_image;
  std::vector<OwnedDecodedImage> decoded_images;
  SubscriptionStats stats;
};

using FrameCallback = std::function<void(const FrameDelivery&)>;
// Moves a raw delivery to a source owner without an additional payload copy.
using OwnedFrameCallback = std::function<void(FrameDelivery)>;

struct SubscriptionState;
struct DepthSubscriptionState;
struct PanoramaSubscriptionState;

class SubscriptionHandle {
 public:
  SubscriptionHandle() = default;
  explicit SubscriptionHandle(std::shared_ptr<SubscriptionState> state);
  SubscriptionHandle(const SubscriptionHandle&) = delete;
  SubscriptionHandle& operator=(const SubscriptionHandle&) = delete;
  SubscriptionHandle(SubscriptionHandle&& other) noexcept;
  SubscriptionHandle& operator=(SubscriptionHandle&& other) noexcept;
  ~SubscriptionHandle();

  void stop();
  // Call outside callbacks. Drain joins callbacks, so application callbacks
  // must return; the transport deadline cannot interrupt user code.
  SubscriptionReport stop(SubscriptionStopMode mode);
  SubscriptionReport report() const;
  bool active() const;
  SubscriptionStats stats() const;

 private:
  std::shared_ptr<SubscriptionState> state_;
};

class DepthSubscriptionHandle {
 public:
  DepthSubscriptionHandle() = default;
  explicit DepthSubscriptionHandle(std::shared_ptr<DepthSubscriptionState> state);
  DepthSubscriptionHandle(const DepthSubscriptionHandle&) = delete;
  DepthSubscriptionHandle& operator=(const DepthSubscriptionHandle&) = delete;
  DepthSubscriptionHandle(DepthSubscriptionHandle&& other) noexcept;
  DepthSubscriptionHandle& operator=(DepthSubscriptionHandle&& other) noexcept;
  ~DepthSubscriptionHandle();

  void stop();
  bool active() const;
  DepthSubscriptionStats stats() const;

 private:
  std::shared_ptr<DepthSubscriptionState> state_;
};

class PanoramaSubscriptionHandle {
 public:
  PanoramaSubscriptionHandle() = default;
  explicit PanoramaSubscriptionHandle(std::shared_ptr<PanoramaSubscriptionState> state);
  PanoramaSubscriptionHandle(const PanoramaSubscriptionHandle&) = delete;
  PanoramaSubscriptionHandle& operator=(const PanoramaSubscriptionHandle&) = delete;
  PanoramaSubscriptionHandle(PanoramaSubscriptionHandle&& other) noexcept;
  PanoramaSubscriptionHandle& operator=(PanoramaSubscriptionHandle&& other) noexcept;
  ~PanoramaSubscriptionHandle();

  void stop();
  bool active() const;
  PanoramaSubscriptionStats stats() const;

 private:
  std::shared_ptr<PanoramaSubscriptionState> state_;
};


}  // namespace orbvi_sdk
