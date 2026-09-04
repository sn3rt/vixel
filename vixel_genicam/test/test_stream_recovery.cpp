#include "vixel_genicam/StreamRecovery.hpp"

#include <gtest/gtest.h>

using namespace std::chrono_literals;

TEST(StreamRecoveryPolicy, RequestsRestartAtThresholdWithinWindow)
{
  vixel_genicam::StreamRecoveryPolicy policy(3, 30s, 30s);
  const auto start = vixel_genicam::StreamRecoveryPolicy::Clock::now();
  policy.record_failure(start);
  policy.record_failure(start + 10s);
  EXPECT_FALSE(policy.restart_requested());
  policy.record_failure(start + 20s);
  EXPECT_TRUE(policy.restart_requested());
  EXPECT_TRUE(policy.degraded());
}

TEST(StreamRecoveryPolicy, IgnoresFailuresOutsideWindow)
{
  vixel_genicam::StreamRecoveryPolicy policy(3, 30s, 30s);
  const auto start = vixel_genicam::StreamRecoveryPolicy::Clock::now();
  policy.record_failure(start);
  policy.record_failure(start + 1s);
  policy.record_failure(start + 31s);
  EXPECT_FALSE(policy.restart_requested());
}

TEST(StreamRecoveryPolicy, EnforcesBackoffAndRecoversAfterHealthyPeriod)
{
  vixel_genicam::StreamRecoveryPolicy policy(2, 30s, 30s, 10min);
  const auto start = vixel_genicam::StreamRecoveryPolicy::Clock::now();
  policy.record_failure(start);
  policy.record_failure(start + 1s);
  ASSERT_TRUE(policy.restart_requested());
  policy.mark_restarted(start + 1s);

  policy.record_failure(start + 2s);
  policy.record_failure(start + 3s);
  EXPECT_FALSE(policy.restart_requested());
  EXPECT_FALSE(policy.record_success(start + 9min));
  EXPECT_TRUE(policy.degraded());
  EXPECT_TRUE(policy.record_success(start + 11min));
  EXPECT_FALSE(policy.degraded());
}
