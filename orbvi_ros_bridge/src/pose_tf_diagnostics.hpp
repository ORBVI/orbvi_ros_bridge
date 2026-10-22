#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace orbvi_ros_bridge {

// Logging policy only: it never changes whether a pose may advance TF.
// The caller serializes observations with the optimized TF mutex.
class PoseTfDiagnostics {
public:
  using Clock = std::chrono::steady_clock;
  struct Report {
    bool warning;
    std::string message;
  };

  std::optional<Report> Observe(bool accepted, const std::string &reason,
                                Clock::time_point now = Clock::now()) {
    if (accepted) {
      ++accepted_;
      if (!healthy_since_) healthy_since_ = now;
    } else {
      ++rejected_;
      failure_pending_ = true;
      healthy_since_.reset();
      last_reason_ = reason;
    }
    if (!failure_pending_ ||
        (last_report_ && now - *last_report_ < std::chrono::seconds(60)))
      return std::nullopt;
    const bool recovered = accepted && healthy_since_ &&
        now - *healthy_since_ >= std::chrono::seconds(5);
    if (accepted && !recovered) return std::nullopt;

    Report report{!recovered,
        std::string(recovered ? "Optimized TF updates resumed" :
                                "Rejected optimized TF samples") +
        ": last_reason=" + last_reason_ +
        "; accepted=" + std::to_string(accepted_) +
        " rejected=" + std::to_string(rejected_) +
        " since previous report; reports limited to once per 60s"};
    last_report_ = now;
    accepted_ = rejected_ = 0;
    if (recovered) failure_pending_ = false;
    return report;
  }

private:
  std::optional<Clock::time_point> last_report_, healthy_since_;
  std::uint64_t accepted_ = 0, rejected_ = 0;
  bool failure_pending_ = false;
  std::string last_reason_;
};

} // namespace orbvi_ros_bridge
