#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace vixel_genicam
{

enum class FrameTimestampRelation
{
  unknown,
  stale,
  matching,
  future,
};

inline FrameTimestampRelation classify_frame_timestamp(
  std::uint64_t observed_ns, std::uint64_t expected_ns, std::uint64_t tolerance_ns)
{
  if (observed_ns == 0 || expected_ns == 0) {
    return FrameTimestampRelation::unknown;
  }
  if (observed_ns < expected_ns && expected_ns - observed_ns > tolerance_ns) {
    return FrameTimestampRelation::stale;
  }
  if (observed_ns > expected_ns && observed_ns - expected_ns > tolerance_ns) {
    return FrameTimestampRelation::future;
  }
  return FrameTimestampRelation::matching;
}

inline std::int64_t median_clock_offset(std::vector<std::int64_t> offsets)
{
  if (offsets.empty()) {throw std::invalid_argument("clock offsets must not be empty");}
  std::sort(offsets.begin(), offsets.end());
  const auto upper = offsets[offsets.size() / 2U];
  if (offsets.size() % 2U != 0U) {return upper;}
  const auto lower = offsets[offsets.size() / 2U - 1U];
  return lower + (upper - lower) / 2;
}

inline bool action_times_conflict(
  std::uint64_t requested_ns, std::uint64_t existing_ns,
  std::uint64_t minimum_separation_ns)
{
  if (requested_ns == existing_ns) {return false;}
  const auto separation = requested_ns > existing_ns ?
    requested_ns - existing_ns : existing_ns - requested_ns;
  return separation < minimum_separation_ns;
}

}  // namespace vixel_genicam
