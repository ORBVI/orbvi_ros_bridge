#include "pose_tf_diagnostics.hpp"
#include <gtest/gtest.h>

using namespace orbvi_ros_bridge;
namespace {
auto At(int seconds) {
  return PoseTfDiagnostics::Clock::time_point{} + std::chrono::seconds(seconds);
}
const std::string reason = "tracking_state=DEGRADED; tracking_reason=recovering";
}

TEST(PoseTfDiagnostics, RepeatedAndFlappingFailuresProduceAtMostOneReportPerMinute) {
  PoseTfDiagnostics diagnostics;
  const auto first = diagnostics.Observe(false, reason, At(0));
  ASSERT_TRUE(first);
  EXPECT_TRUE(first->warning);
  for (int t = 1; t < 60; ++t)
    EXPECT_FALSE(diagnostics.Observe(t % 5 != 0, reason, At(t)));
  const auto summary = diagnostics.Observe(false, reason, At(60));
  ASSERT_TRUE(summary);
  EXPECT_TRUE(summary->warning);
  EXPECT_NE(summary->message.find("accepted=48 rejected=12"), std::string::npos);
}

TEST(PoseTfDiagnostics, RecoveryRequiresStableTrackingAndUsesInfo) {
  PoseTfDiagnostics diagnostics;
  ASSERT_TRUE(diagnostics.Observe(false, reason, At(0)));
  EXPECT_FALSE(diagnostics.Observe(false, reason, At(58)));
  EXPECT_FALSE(diagnostics.Observe(true, "", At(59)));
  EXPECT_FALSE(diagnostics.Observe(true, "", At(60)));
  const auto recovered = diagnostics.Observe(true, "", At(64));
  ASSERT_TRUE(recovered);
  EXPECT_FALSE(recovered->warning);
  EXPECT_NE(recovered->message.find("resumed"), std::string::npos);
  EXPECT_FALSE(diagnostics.Observe(true, "", At(130)));
  EXPECT_TRUE(diagnostics.Observe(false, "duplicate_timestamp", At(131)));
}

TEST(PoseTfDiagnostics, HealthyStreamIsSilentAndUsesMonotonicTime) {
  PoseTfDiagnostics diagnostics;
  for (int t = 0; t < 200; ++t)
    EXPECT_FALSE(diagnostics.Observe(true, "", At(t)));
  ASSERT_TRUE(diagnostics.Observe(false, reason, At(200)));
  // No ROS/board clock is used, including when board time is old or resets.
  EXPECT_FALSE(diagnostics.Observe(false, "out_of_order_timestamp", At(259)));
  const auto next = diagnostics.Observe(false, "out_of_order_timestamp", At(260));
  ASSERT_TRUE(next);
  EXPECT_NE(next->message.find("out_of_order_timestamp"), std::string::npos);
}
