#include "vixel_recorder/SequenceTiming.hpp"

#include <gtest/gtest.h>

using namespace std::chrono_literals;

TEST(SequenceTiming, RejectsOnlyKnownUnsafeIntervals)
{
  EXPECT_TRUE(vixel_recorder::capture_interval_supported(200, 0));
  EXPECT_TRUE(vixel_recorder::capture_interval_supported(200, 200));
  EXPECT_FALSE(vixel_recorder::capture_interval_supported(199, 200));
}

TEST(SequenceTiming, UsesConfiguredLeadWhenOneActionFits)
{
  EXPECT_EQ(vixel_recorder::sequence_dispatch_lead(150ms, 200ms, 1), 150ms);
}

TEST(SequenceTiming, DoesNotOverfillUnknownSingleEntryQueue)
{
  EXPECT_EQ(vixel_recorder::sequence_dispatch_lead(750ms, 200ms, 1), 180ms);
}

TEST(SequenceTiming, UsesReportedActionQueueCapacity)
{
  EXPECT_EQ(vixel_recorder::sequence_dispatch_lead(750ms, 200ms, 4), 750ms);
}
