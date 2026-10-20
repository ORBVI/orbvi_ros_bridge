#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <memory>
#include <functional>

#if defined(__GNUC__)
#define ORBVI_LOGGING_API __attribute__((visibility("default")))
#else
#define ORBVI_LOGGING_API
#endif

namespace orbvi::logging {

enum class Level {
  kTrace,
  kDebug,
  kInfo,
  kWarning,
  kError,
  kCritical,
  kOff,
};

struct SourceLocation {
  const char *file = "";
  int line = 0;
  const char *function = "";
};

using Sink = std::function<void(Level, std::string_view component, std::string_view message)>;
struct Options {
  std::string component;
  Level level = Level::kInfo;
  // Persist diagnostic events promptly without flushing every info message.
  Level flush_level = Level::kWarning;
  bool stderr_enabled = true;
  bool file_enabled = false;
  std::string file_directory;
  std::size_t file_max_size_bytes = 10U * 1024U * 1024U;
  // Total retained files, including the active file.
  std::size_t file_max_files = 5;
  // Optional application output. SDK invokes it on the diagnostic worker;
  // synchronous Logger::Write invokes it on that caller. Views last this call.
  Sink sink;
};

ORBVI_LOGGING_API Level ParseLevel(std::string_view text, Level fallback,
                 bool *valid = nullptr) noexcept;

ORBVI_LOGGING_API bool Initialize(const Options &options, std::string *reason = nullptr) noexcept;
ORBVI_LOGGING_API bool InitializeFromEnvironment(std::string component,
                               std::string *reason = nullptr) noexcept;
ORBVI_LOGGING_API bool ShouldLog(Level level) noexcept;
ORBVI_LOGGING_API bool ShouldLogThrottled(Level level, SourceLocation source,
                        std::chrono::nanoseconds period) noexcept;
ORBVI_LOGGING_API void Log(Level level, SourceLocation source, std::string_view message) noexcept;
ORBVI_LOGGING_API void Flush() noexcept;
ORBVI_LOGGING_API void Shutdown() noexcept;

// Deferred submissions never wait for a sink or queue space. Each instance owns
// a 256-record queue per prepared logger, each message capped at 2048 bytes.
// These global helpers use only the application default logger.
struct DeferredReport {
  std::uint64_t accepted = 0, processed = 0, dropped = 0, truncated = 0;
  std::size_t pending = 0, high_water = 0;
};
ORBVI_LOGGING_API bool PrepareDeferred() noexcept;
ORBVI_LOGGING_API bool LogDeferred(Level level, SourceLocation source, std::string_view message) noexcept;
ORBVI_LOGGING_API DeferredReport DeferredStatus() noexcept;
// Waits only for the prefix accepted before this call. False means still pending.
ORBVI_LOGGING_API bool DrainDeferred(std::chrono::milliseconds timeout) noexcept;
class ORBVI_LOGGING_API Logger final {
 public:
  static std::shared_ptr<Logger> Create(const Options& options, std::string* reason = nullptr) noexcept;
  ~Logger();
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;
  bool ShouldLog(Level level) const noexcept;
  void SetLevel(Level level) noexcept;
  void Write(Level level, SourceLocation source, std::string_view message) noexcept;
  bool PrepareDeferred() noexcept; // Call from a control thread before submitting.
  bool WriteDeferred(Level level, SourceLocation source, std::string_view message) noexcept;
  DeferredReport DeferredStatus() const noexcept;
  bool DrainDeferred(std::chrono::milliseconds timeout) noexcept;
  void Flush() noexcept;
 private:
  struct Impl;
  explicit Logger(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
  friend bool Initialize(const Options&, std::string*) noexcept;
};
// Null before application Initialize(). Sharing this handle is explicit;
// constructing a different Logger does not change it.
ORBVI_LOGGING_API std::shared_ptr<Logger> DefaultLogger() noexcept;

struct DeferredTag {};

class ORBVI_LOGGING_API LogEntry final {
public:
  LogEntry(Level level, SourceLocation source) noexcept;
  LogEntry(Level level, SourceLocation source,
           std::chrono::nanoseconds throttle_period) noexcept;
  LogEntry(Level level, SourceLocation source, DeferredTag) noexcept;
  LogEntry(std::shared_ptr<Logger> logger, Level level, SourceLocation source, DeferredTag) noexcept;
  ~LogEntry() noexcept;

  LogEntry(const LogEntry &) = delete;
  LogEntry &operator=(const LogEntry &) = delete;
  LogEntry(LogEntry &&) = delete;
  LogEntry &operator=(LogEntry &&) = delete;

  template <typename T> LogEntry &operator<<(T &&value) noexcept {
    if (enabled_ && stream_.has_value()) {
      try {
        *stream_ << std::forward<T>(value);
      } catch (...) {
        enabled_ = false;
        stream_.reset();
      }
    }
    return *this;
  }

  using StreamManipulator = std::ostream &(*)(std::ostream &);
  LogEntry &operator<<(StreamManipulator manipulator) noexcept;

private:
  Level level_;
  SourceLocation source_;
  bool enabled_ = false;
  bool deferred_ = false;
  std::shared_ptr<Logger> logger_;
  std::optional<std::ostringstream> stream_;
};

} // namespace orbvi::logging

#define ORBVI_LOG_ENTRY(level)                                                 \
  ::orbvi::logging::LogEntry(                                                  \
      (level), ::orbvi::logging::SourceLocation{__FILE__, __LINE__, __func__})

#define ORBVI_LOG_TRACE() ORBVI_LOG_ENTRY(::orbvi::logging::Level::kTrace)
#define ORBVI_LOG_DEBUG() ORBVI_LOG_ENTRY(::orbvi::logging::Level::kDebug)
#define ORBVI_LOG_INFO() ORBVI_LOG_ENTRY(::orbvi::logging::Level::kInfo)
#define ORBVI_LOG_WARNING() ORBVI_LOG_ENTRY(::orbvi::logging::Level::kWarning)
#define ORBVI_LOG_ERROR() ORBVI_LOG_ENTRY(::orbvi::logging::Level::kError)
#define ORBVI_LOG_CRITICAL() ORBVI_LOG_ENTRY(::orbvi::logging::Level::kCritical)

#define ORBVI_LOG_THROTTLED_ENTRY(level, seconds)                              \
  ::orbvi::logging::LogEntry(                                                  \
      (level), ::orbvi::logging::SourceLocation{__FILE__, __LINE__, __func__}, \
      std::chrono::duration_cast<std::chrono::nanoseconds>(                    \
          std::chrono::duration<double>(seconds)))

#define ORBVI_LOG_TRACE_THROTTLED(seconds)                                     \
  ORBVI_LOG_THROTTLED_ENTRY(::orbvi::logging::Level::kTrace, seconds)
#define ORBVI_LOG_DEBUG_THROTTLED(seconds)                                     \
  ORBVI_LOG_THROTTLED_ENTRY(::orbvi::logging::Level::kDebug, seconds)
#define ORBVI_LOG_INFO_THROTTLED(seconds)                                      \
  ORBVI_LOG_THROTTLED_ENTRY(::orbvi::logging::Level::kInfo, seconds)
#define ORBVI_LOG_WARNING_THROTTLED(seconds)                                   \
  ORBVI_LOG_THROTTLED_ENTRY(::orbvi::logging::Level::kWarning, seconds)
#define ORBVI_LOG_ERROR_THROTTLED(seconds)                                     \
  ORBVI_LOG_THROTTLED_ENTRY(::orbvi::logging::Level::kError, seconds)
#define ORBVI_LOG_CRITICAL_THROTTLED(seconds)                                  \
  ORBVI_LOG_THROTTLED_ENTRY(::orbvi::logging::Level::kCritical, seconds)

#define ORBVI_LOG_DEFERRED(level) \
  ::orbvi::logging::LogEntry((level), \
      ::orbvi::logging::SourceLocation{__FILE__, __LINE__, __func__}, \
      ::orbvi::logging::DeferredTag{})
