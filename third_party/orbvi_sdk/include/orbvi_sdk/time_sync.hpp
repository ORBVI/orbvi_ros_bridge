#pragma once
#include "orbvi_sdk/frame.hpp"
#include "orbvi_time_contract/status.hpp"

namespace orbvi_sdk {
using TimeSyncSupport = orbvi_time_contract::Support;
using TimeSyncState = orbvi_time_contract::State;
using ClockSyncStatus = orbvi_time_contract::Channel;
using TimeSyncStatus = orbvi_time_contract::Status;
using TimeSyncCapabilities = orbvi_time_contract::Capabilities;
// Parses capture-time evidence. Never queries a live device or promotes legacy
// internal timing flags to external clock lock. Product identity must be known.
Result<TimeSyncStatus> ParseFrameTimeSyncStatus(const FrameView& frame);
// Shared policy for consumers requiring only a valid local device time base.
// External UTC/PTP lock is deliberately not an input to this decision.
Result<void> ValidateLocalFrameTime(const FrameView& frame);
}  // namespace orbvi_sdk
