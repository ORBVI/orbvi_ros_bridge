#pragma once
#include <memory>
#include <string>
#include "orbvi_camera_contract/product.hpp"
#include "orbvi_sdk/compatibility.hpp"
#include "orbvi_sdk/status.hpp"
#include "orbvi_sdk/time_sync.hpp"
#include "orbvi_sdk/transport_options.hpp"

namespace orbvi_sdk {
namespace internal { struct ContractBinding; class DeviceSession; }
// Unknown remains unknown. Known contradictory declarations are errors. The
// legacy alias is interpreted here, never independently by a consumer.
Result<orbvi_camera_contract::Product> ResolveDeviceProduct(
    PeerContract peer, const std::string& model, const std::string& hardware_profile = {});

// Immutable session capability. Copies retain the same frozen binding; a new
// device epoch requires another contract, never a mutation of this one.
class DeviceContract {
 public:
  DeviceContract() = default;
  orbvi_camera_contract::Product product() const;
  PeerContract peer() const;
  TransportProfile transport() const;
  std::string model() const;
  TimeSyncCapabilities timeSyncCapabilities() const;
  Result<void> admit(StreamId stream) const;
 private:
  friend class Client;
  friend class internal::DeviceSession;
  friend Result<DeviceContract> ResolveDeviceContract(const std::string&, CompatibilityMode, TransportProfile);
  explicit DeviceContract(std::shared_ptr<const internal::ContractBinding> binding)
      : binding_(std::move(binding)) {}
  std::shared_ptr<const internal::ContractBinding> binding_;
};
Result<DeviceContract> ResolveDeviceContract(const std::string& device_info_json,
    CompatibilityMode mode = CompatibilityMode::Current,
    TransportProfile transport = TransportProfile::NetworkNetwork);
}  // namespace orbvi_sdk
