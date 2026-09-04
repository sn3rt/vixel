#pragma once

#include <chrono>
#include <cstddef>
#include <deque>

namespace vixel_genicam
{

class StreamRecoveryPolicy
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  StreamRecoveryPolicy(
    std::size_t error_threshold, std::chrono::milliseconds error_window,
    std::chrono::milliseconds restart_backoff,
    std::chrono::milliseconds healthy_reset = std::chrono::minutes(10))
  : error_threshold_(error_threshold), error_window_(error_window),
    restart_backoff_(restart_backoff), healthy_reset_(healthy_reset)
  {
  }

  void record_failure(TimePoint now)
  {
    if (last_failure_ != TimePoint{} && now - last_failure_ >= healthy_reset_) {
      failures_.clear();
    }
    last_failure_ = now;
    degraded_ = true;
    while (!failures_.empty() && now - failures_.front() > error_window_) {
      failures_.pop_front();
    }
    failures_.push_back(now);
    const bool outside_backoff = last_restart_ == TimePoint{} ||
      now - last_restart_ >= restart_backoff_;
    if (outside_backoff && failures_.size() >= error_threshold_) {
      restart_requested_ = true;
    }
  }

  bool record_success(TimePoint now)
  {
    if (last_failure_ == TimePoint{} || now - last_failure_ < healthy_reset_) {
      return false;
    }
    failures_.clear();
    degraded_ = false;
    restart_requested_ = false;
    return true;
  }

  void mark_restarted(TimePoint now)
  {
    restart_requested_ = false;
    failures_.clear();
    last_restart_ = now;
    degraded_ = true;
  }

  bool restart_requested() const {return restart_requested_;}
  bool degraded() const {return degraded_;}

private:
  std::size_t error_threshold_;
  std::chrono::milliseconds error_window_;
  std::chrono::milliseconds restart_backoff_;
  std::chrono::milliseconds healthy_reset_;
  std::deque<TimePoint> failures_;
  TimePoint last_failure_{};
  TimePoint last_restart_{};
  bool degraded_{false};
  bool restart_requested_{false};
};

}  // namespace vixel_genicam
