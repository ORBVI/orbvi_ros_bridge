#pragma once
#include <orbvi/logging/logging.hpp>

namespace orbvi_sdk {
// SDK-wide default, shared by Clients and captures. It is independent of the
// application's orbvi::logging default and initially writes Info+ to stderr.
std::shared_ptr<orbvi::logging::Logger> GetLogInstance();
bool ConfigureLogging(orbvi::logging::Options options, std::string* reason = nullptr);
// Explicit application takeover. Null restores the SDK's default instance.
// Call from a control thread; throws if its diagnostic worker cannot start.
// Keep the previous handle and drain it before releasing callback-owned state.
void SetLogInstance(std::shared_ptr<orbvi::logging::Logger> instance);
} // namespace orbvi_sdk
