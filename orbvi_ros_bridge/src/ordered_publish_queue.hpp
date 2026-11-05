#pragma once

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

namespace orbvi_ros_bridge {

// FIFO within each topic; independent topics may run concurrently. At most
// capacity pending tasks plus one in-flight task per worker are retained.
// Start/Stop are called serially by the owner; Enqueue may run concurrently.
class OrderedPublishQueue {
 public:
  struct EnqueueResult {
    bool accepted = false;
    bool dropped_oldest = false;
    std::uint64_t total_dropped = 0;
  };

  ~OrderedPublishQueue() { Stop(); }

  void Start(std::size_t worker_count, std::size_t capacity) {
    Stop();
    if (worker_count == 0) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      capacity_ = std::max<std::size_t>(1, capacity);
      dropped_ = 0;
      stopping_ = false;
    }
    try {
      for (std::size_t i = 0; i < worker_count; ++i)
        workers_.emplace_back([this] { Run(); });
    } catch (...) {
      Stop();
      throw;
    }
  }

  void Stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      pending_.clear();
    }
    changed_.notify_all();
    for (auto& worker : workers_) if (worker.joinable()) worker.join();
    workers_.clear();
  }

  EnqueueResult Enqueue(std::string topic, std::function<void()> publish) {
    EnqueueResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) return result;
      result.accepted = true;
      if (pending_.size() >= capacity_) {
        pending_.pop_front();
        result.dropped_oldest = true;
        ++dropped_;
      }
      result.total_dropped = dropped_;
      pending_.push_back({std::move(topic), std::move(publish)});
    }
    changed_.notify_one();
    return result;
  }

 private:
  struct Task {
    std::string topic;
    std::function<void()> publish;
  };

  auto NextReady() {
    return std::find_if(pending_.begin(), pending_.end(), [this](const Task& task) {
      return active_.count(task.topic) == 0;
    });
  }

  void Run() {
    while (true) {
      Task task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock, [this] { return stopping_ || NextReady() != pending_.end(); });
        if (stopping_) return;
        auto ready = NextReady();
        task = std::move(*ready);
        pending_.erase(ready);
        active_.insert(task.topic);
      }
      task.publish();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        active_.erase(task.topic);
      }
      changed_.notify_all();
    }
  }

  std::mutex mutex_;
  std::condition_variable changed_;
  std::deque<Task> pending_;
  std::unordered_set<std::string> active_;
  std::vector<std::thread> workers_;
  std::size_t capacity_ = 1;
  std::uint64_t dropped_ = 0;
  bool stopping_ = true;
};
}  // namespace orbvi_ros_bridge
