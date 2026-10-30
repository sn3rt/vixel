#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace vixel_recorder
{

inline bool capture_interval_supported(
  std::uint32_t requested_interval_ms, std::uint32_t minimum_interval_ms)
{
  return minimum_interval_ms == 0 || requested_interval_ms >= minimum_interval_ms;
}

inline std::chrono::milliseconds sequence_dispatch_lead(
  std::chrono::milliseconds configured_lead, std::chrono::milliseconds interval,
  std::uint32_t action_queue_size)
{
  constexpr auto required_camera_margin = std::chrono::milliseconds(20);
  const auto queue_depth = std::max<std::uint32_t>(1, action_queue_size);
  const auto queue_window = interval * queue_depth;
  const auto maximum_safe_lead = queue_window > required_camera_margin ?
    queue_window - required_camera_margin : required_camera_margin;
  return std::max(required_camera_margin, std::min(configured_lead, maximum_safe_lead));
}

inline std::chrono::milliseconds group_dispatch_offset(
  std::chrono::milliseconds interval, std::size_t group_count,
  std::size_t group_index)
{
  if (group_count <= 1 || group_index == 0 || interval.count() <= 0) {
    return std::chrono::milliseconds(0);
  }
  const auto phase = std::max<std::int64_t>(
    1, interval.count() / static_cast<std::int64_t>(group_count));
  return std::chrono::milliseconds(
    phase * static_cast<std::int64_t>(group_index));
}

}  // namespace vixel_recorder
