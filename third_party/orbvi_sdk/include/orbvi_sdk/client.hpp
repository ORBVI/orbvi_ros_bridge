#pragma once
#include "orbvi_sdk/device_contract.hpp"
#include "orbvi_sdk/client_options.hpp"
#include "orbvi_sdk/device_info.hpp"
#include "orbvi_sdk/calibration_install.hpp"
#include "orbvi_sdk/subscription_handle.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "orbvi_sdk/frame.hpp"
#include "orbvi_sdk/subscription.hpp"
#include "orbvi_sdk/lidar_calibration.hpp"
#include "orbvi_sdk/livox.hpp"
#include "orbvi_sdk/image.hpp"
#include "orbvi_sdk/depth.hpp"
#include "orbvi_sdk/panorama.hpp"
#include "orbvi_sdk/transport_options.hpp"
#include "orbvi_sdk/uvc_contract.hpp"

namespace orbvi_sdk {

class BulkReader;
class UvcReader;

namespace detail {
class BulkOtaRuntime;
class ClientTestPeer;
class SessionContractAccess;
}

class Client {
 public:
  explicit Client(ClientOptions options);
  ~Client();

  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;
  Client(Client&&) noexcept;
  Client& operator=(Client&&) noexcept;

  Result<void> connect();
  void disconnect();
  bool connected() const;
  Result<PeerCompatibility> getCompatibility() const;
  Result<DeviceContract> getContract() const;

  Result<DeviceInfo> getDeviceInfo();
  // Network / Vendor Bulk provisioning. UVC-only has no control endpoint.
  Result<DeviceSnStatus> getDeviceSn();
  // Exactly 14 UTF-8 characters, no naming-pattern checks. Atomically persists
  // the SN; active DeviceInfo/USB/calibration identity changes after reboot.
  Result<DeviceSnStatus> setDeviceSn(const std::string& device_sn);
  Result<DeviceStatus> getStatus();
  // Query the 4VL estimator epoch without changing its state.
  Result<AlgorithmStatus> getAlgorithmStatus();
  // Explicitly reinitialize the 4VL estimator and its origin. Sensor owners
  // keep running; normal mapping start/pause/stop must not call this implicitly.
  // The application saves/discards pending data and waits for valid new poses.
  // Success means runtime startup, not completion of estimator initialization.
  Result<AlgorithmStatus> resetAlgorithm();
  Result<TimeStatus> getTimeStatus();
  Result<TimeSyncStatus> getTimeSyncStatus();
  // Nonempty version checks the device's current snapshot on every call,
  // bypassing historical frame caches; any different version is rejected.
  Result<CalibrationStatus> getCalibration(const std::string& version = "");
  // Always fetches a fresh installed-collection snapshot. Older firmware that
  // lacks /v1/calibration/installed returns CalibrationMissing. Does not change
  // the layout of the existing CalibrationStatus value type.
  Result<InstalledCalibration> getInstalledCalibration();
  // Network-only. Uploads one bounded JSON package; acceptance is not success.
  // Never automatically retries an ambiguous POST. Keep operation_id to query
  // after the service restarts; a transport timeout does not prove rejection.
  Result<CalibrationInstallAcceptance> installCalibrationPackage(const std::string& package_path);
  // Uploads exactly once, reports progress outside internal locks, waits for
  // completion and verifies readback against the uploaded bytes. Cancels old
  // subscriptions and clears frame/calibration caches after acceptance; callers
  // resubscribe after success. Errors after acceptance retain operation_id in
  // the message; save it immediately in the accepted callback for recovery.
  Result<CalibrationInstallResult> installCalibrationPackageAndWait(
      const std::string& package_path,
      CalibrationInstallProgressCallback progress = {},
      CalibrationInstallWaitOptions options = {});
  Result<CalibrationInstallStatus> getCalibrationInstallStatus(const std::string& operation_id);
  // Returns board-declared LiDAR IMU units, separate from factory calibration.
  Result<LidarImuParameters> getLidarImuParameters();
  // Reads the LiDAR diagnostic snapshot, preserving available camera and
  // manufacturer IMU transforms even when camera files are missing. Success
  // does not imply readiness: check status == "READY" for a complete camera
  // bundle and imu_extrinsic for the independent manufacturer transform.
  Result<LidarCalibration> getLidarCalibration(const std::string& version = "");
  Result<OtaProgress> getOtaProgress();
  Result<std::string> getOtaLog();
  Result<OtaCommandResult> beginOta(const std::string& package_path);
  Result<OtaCommandResult> uploadOtaPackage(const std::string& package_path);
  Result<OtaCommandResult> cancelOta();
  Result<OtaCommandResult> commitOta();

  Result<SubscriptionHandle> subscribe(StreamId stream, FrameCallback callback);
  Result<SubscriptionHandle> subscribe(
      StreamId stream,
      SubscribeOptions options,
      FrameCallback callback);
  // Public ordered raw delivery for native applications, bridges and recorders.
  // Requires continuous streaming; decode/sample fallback are disabled.
  Result<SubscriptionHandle> subscribe(
      StreamId stream, SubscribeOptions options,
      OrderedDeliveryOptions delivery, FrameCallback callback);
  Result<SubscriptionHandle> subscribeOwned(
      StreamId stream, SubscribeOptions options,
      OrderedDeliveryOptions delivery, OwnedFrameCallback callback);
  Result<DepthMap> getDepthMap(const DepthGenerationOptions& options = {});
  Result<DepthSubscriptionHandle> subscribeDepthMap(DepthFrameCallback callback);
  Result<DepthSubscriptionHandle> subscribeDepthMap(
      DepthSubscribeOptions options,
      DepthFrameCallback callback);
  Result<OwnedPanoramaImage> getPanorama(const PanoramaStitchOptions& options = {});
  Result<OwnedPanoramaImage> getPanorama(
      PanoramaSourceMode source_mode,
      const PanoramaStitchOptions& options = {});
  Result<PanoramaSubscriptionHandle> subscribePanorama(PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSubscribeOptions options,
      PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSourceMode source_mode,
      PanoramaSubscribeOptions options,
      PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSubscribeOptions options,
      PanoramaDepthProvider depth_provider,
      PanoramaFrameCallback callback);
  Result<PanoramaSubscriptionHandle> subscribePanorama(
      PanoramaSourceMode source_mode,
      PanoramaSubscribeOptions options,
      PanoramaDepthProvider depth_provider,
      PanoramaFrameCallback callback);

 private:
  friend class detail::ClientTestPeer;
  friend class detail::SessionContractAccess;

  // Private production-path test seam. Tests inject a BulkReader carrying a
  // fake UsbBackend; normal callers always use the public constructor and the
  // default libusb backend.
  Client(
      ClientOptions options,
      std::function<void(BulkReader*)> prepare_bulk_reader_for_open,
      std::function<void(detail::BulkOtaRuntime*)>
          prepare_bulk_ota_for_open = {});
  // Private production-path test seam for a fake V4L2 backend. Normal callers
  // always use the public constructor and dynamic composite discovery.
  Client(
      ClientOptions options,
      std::function<void(UvcReader*)> prepare_uvc_reader_for_open);
  // Private pure-USB production-path test seam. It lets one test exercise the
  // real UVC + Vendor Bulk profile while injecting all three hardware-facing
  // backends; normal callers cannot construct mixed transport states.
  Client(
      ClientOptions options,
      std::function<void(BulkReader*)> prepare_bulk_reader_for_open,
      std::function<void(detail::BulkOtaRuntime*)>
          prepare_bulk_ota_for_open,
      std::function<void(UvcReader*)> prepare_uvc_reader_for_open);

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace orbvi_sdk
