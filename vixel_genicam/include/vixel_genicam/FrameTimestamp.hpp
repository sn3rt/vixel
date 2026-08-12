#pragma once

#include <cstdint>

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

}  // namespace vixel_genicam
