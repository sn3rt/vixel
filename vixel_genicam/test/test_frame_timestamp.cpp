#include "vixel_genicam/FrameTimestamp.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

using vixel_genicam::FrameTimestampRelation;
using vixel_genicam::action_times_conflict;
using vixel_genicam::classify_frame_timestamp;
using vixel_genicam::median_clock_offset;

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

TEST(FrameTimestamp, UsesOneDomainOffsetAcrossEverySelection)
{
  EXPECT_EQ(median_clock_offset({120, 80, 100}), 100);
  EXPECT_EQ(median_clock_offset({140, 80, 100, 120}), 110);
}

TEST(FrameTimestamp, RejectsAnEmptySynchronizationDomain)
{
  EXPECT_THROW(median_clock_offset({}), std::invalid_argument);
}

TEST(FrameTimestamp, MergesExactActionTargetsAndRejectsOnlyTooCloseTargets)
{
  EXPECT_FALSE(action_times_conflict(1'000'000, 1'000'000, 250'000));
  EXPECT_TRUE(action_times_conflict(1'000'000, 1'249'999, 250'000));
  EXPECT_FALSE(action_times_conflict(1'000'000, 1'250'000, 250'000));
}

TEST(FrameTimestamp, ComparesActionTargetsWithoutUnsignedOverflow)
{
  EXPECT_FALSE(action_times_conflict(
      0, std::numeric_limits<std::uint64_t>::max(), 250'000));
}
