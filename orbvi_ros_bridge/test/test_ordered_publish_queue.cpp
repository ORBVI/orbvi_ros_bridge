#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <numeric>
#include <vector>

#include "ordered_publish_queue.hpp"

namespace {
using orbvi_ros_bridge::OrderedPublishQueue;
using namespace std::chrono_literals;

// Always release blocked workers before the queue is destroyed, including on
// assertion failure. Tests use events rather than scheduler-dependent sleeps.
struct Gate {
  std::promise<void> entered, release;
  std::shared_future<void> released = release.get_future().share();
  ~Gate() { if (!opened) Open(); }
  void Open() { release.set_value(); opened = true; }
  void Block() { entered.set_value(); released.wait(); }
  bool opened = false;
};

TEST(OrderedPublishQueue, SameTopicIsFifoWhileOtherTopicsRunConcurrently) {
  OrderedPublishQueue queue;
  Gate first;
  queue.Start(8, 64);
  std::mutex mutex;
  std::vector<int> observed;
  std::atomic<bool> second_started{false};
  queue.Enqueue("imu", [&] { first.Block(); std::lock_guard<std::mutex> lock(mutex); observed.push_back(0); });
  EXPECT_EQ(first.entered.get_future().wait_for(2s), std::future_status::ready);
  std::promise<void> last, other_topic;
  for (int i = 1; i <= 20; ++i) {
    queue.Enqueue("imu", [&, i] {
      second_started = true;
      std::lock_guard<std::mutex> lock(mutex);
      observed.push_back(i);
      if (i == 20) last.set_value();
    });
  }
  queue.Enqueue("image", [&] { other_topic.set_value(); });
  EXPECT_EQ(other_topic.get_future().wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(second_started);
  first.Open();
  EXPECT_EQ(last.get_future().wait_for(2s), std::future_status::ready);
  queue.Stop();
  std::vector<int> expected(21);
  std::iota(expected.begin(), expected.end(), 0);
  EXPECT_EQ(observed, expected);
}

TEST(OrderedPublishQueue, OverflowDropsOldestPendingWithoutReorderingSurvivors) {
  OrderedPublishQueue queue;
  Gate first;
  queue.Start(1, 2);
  queue.Enqueue("imu", [&] { first.Block(); });
  EXPECT_EQ(first.entered.get_future().wait_for(2s), std::future_status::ready);
  std::vector<int> observed;
  std::promise<void> last;
  EXPECT_FALSE(queue.Enqueue("imu", [&] { observed.push_back(1); }).dropped_oldest);
  EXPECT_FALSE(queue.Enqueue("imu", [&] { observed.push_back(2); }).dropped_oldest);
  auto result = queue.Enqueue("imu", [&] { observed.push_back(3); last.set_value(); });
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.dropped_oldest);
  EXPECT_EQ(result.total_dropped, 1u);
  first.Open();
  EXPECT_EQ(last.get_future().wait_for(2s), std::future_status::ready);
  queue.Stop();
  EXPECT_EQ(observed, (std::vector<int>{2, 3}));
}

struct DestructionSignal {
  std::promise<void> dropped;
  ~DestructionSignal() { dropped.set_value(); }
};

TEST(OrderedPublishQueue, StopDiscardsPendingAndWaitsForInflightThenCanRestart) {
  OrderedPublishQueue queue;
  Gate first;
  queue.Start(2, 4);
  queue.Enqueue("imu", [&] { first.Block(); });
  EXPECT_EQ(first.entered.get_future().wait_for(2s), std::future_status::ready);
  std::atomic<bool> pending_ran{false};
  auto signal = std::make_shared<DestructionSignal>();
  auto dropped = signal->dropped.get_future();
  queue.Enqueue("imu", [&, signal] { pending_ran = true; });
  signal.reset();
  auto stopped = std::async(std::launch::async, [&] { queue.Stop(); });
  EXPECT_EQ(dropped.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(stopped.wait_for(0s), std::future_status::timeout);
  first.Open();
  EXPECT_EQ(stopped.wait_for(2s), std::future_status::ready);
  stopped.get();
  EXPECT_FALSE(pending_ran);
  EXPECT_FALSE(queue.Enqueue("imu", [] {}).accepted);
  queue.Start(2, 4);
  std::promise<void> restarted;
  EXPECT_TRUE(queue.Enqueue("imu", [&] { restarted.set_value(); }).accepted);
  EXPECT_EQ(restarted.get_future().wait_for(2s), std::future_status::ready);
  queue.Stop();
}
}  // namespace
