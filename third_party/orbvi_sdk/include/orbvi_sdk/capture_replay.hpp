#pragma once
#include "orbvi_sdk/capture.hpp"
#include "orbvi_sdk/mcap_reading.hpp"
namespace orbvi_sdk {
// Preflight validates the full file and freezes its actual SDK stream manifest.
// Replay uses the same outlet; ordered consumers apply backpressure instead
// of dropping frames. No live device is opened or controlled.
Result<std::unique_ptr<CaptureSource>> OpenMcapCaptureSource(
    const std::string& path, const McapReadingOptions& options = {});
}
