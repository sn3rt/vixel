#include "vixel_genicam/TriggerReadiness.hpp"

#include <gtest/gtest.h>

#include <chrono>

using namespace std::chrono_literals;

TEST(TriggerReadiness, ReadyImmediatelyDoesNotSleep)
{
  using Clock = std::chrono::steady_clock;
  auto now = Clock::time_point{};
  int sleeps = 0;

  EXPECT_TRUE(vixel_genicam::wait_for_trigger_armed(
      now + 100ms, Clock::duration(10ms), []() {return true;},
      [&now]() {return now;},
      [&now, &sleeps](const auto delay) {now += delay; ++sleeps;}));
  EXPECT_EQ(sleeps, 0);
}

TEST(TriggerReadiness, PollsUntilCameraRearms)
{
  using Clock = std::chrono::steady_clock;
  auto now = Clock::time_point{};
  int polls = 0;

  EXPECT_TRUE(vixel_genicam::wait_for_trigger_armed(
      now + 100ms, Clock::duration(10ms), [&polls]() {return ++polls == 3;},
      [&now]() {return now;}, [&now](const auto delay) {now += delay;}));
  EXPECT_EQ(polls, 3);
  EXPECT_EQ(now.time_since_epoch(), 20ms);
}

TEST(TriggerReadiness, TimeoutDoesNotOversleepDeadline)
{
  using Clock = std::chrono::steady_clock;
  auto now = Clock::time_point{};
  int polls = 0;

  EXPECT_FALSE(vixel_genicam::wait_for_trigger_armed(
      now + 25ms, Clock::duration(10ms), [&polls]() {++polls; return false;},
      [&now]() {return now;}, [&now](const auto delay) {now += delay;}));
  EXPECT_EQ(polls, 4);
  EXPECT_EQ(now.time_since_epoch(), 25ms);
}
