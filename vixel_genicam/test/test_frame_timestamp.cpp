#include "vixel_genicam/FrameTimestamp.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

using vixel_genicam::FrameTimestampRelation;
using vixel_genicam::classify_frame_timestamp;

TEST(FrameTimestamp, TreatsMissingTimestampsAsUnknown)
{
  EXPECT_EQ(classify_frame_timestamp(0, 1'000'000, 100), FrameTimestampRelation::unknown);
  EXPECT_EQ(classify_frame_timestamp(1'000'000, 0, 100), FrameTimestampRelation::unknown);
}

TEST(FrameTimestamp, AcceptsValuesInsideInclusiveTolerance)
{
  EXPECT_EQ(classify_frame_timestamp(999'900, 1'000'000, 100),
    FrameTimestampRelation::matching);
  EXPECT_EQ(classify_frame_timestamp(1'000'100, 1'000'000, 100),
    FrameTimestampRelation::matching);
}

TEST(FrameTimestamp, RejectsStaleAndFutureFrames)
{
  EXPECT_EQ(classify_frame_timestamp(999'899, 1'000'000, 100),
    FrameTimestampRelation::stale);
  EXPECT_EQ(classify_frame_timestamp(1'000'101, 1'000'000, 100),
    FrameTimestampRelation::future);
}

TEST(FrameTimestamp, AvoidsUnsignedOverflowAtClockLimits)
{
  EXPECT_EQ(classify_frame_timestamp(
      std::numeric_limits<std::uint64_t>::max(),
      std::numeric_limits<std::uint64_t>::max() - 50, 100),
    FrameTimestampRelation::matching);
  EXPECT_EQ(classify_frame_timestamp(
      50, std::numeric_limits<std::uint64_t>::max() - 50, 100),
    FrameTimestampRelation::stale);
}
