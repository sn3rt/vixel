#include "vixel_recorder/ManagedWorkers.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

TEST(ManagedWorkers, ReapsCompletedThreads)
{
  vixel_recorder::ManagedWorkers workers;
  std::atomic_bool completed{false};
  ASSERT_TRUE(workers.start([&completed]() {completed = true;}));
  for (int attempt = 0; attempt < 100 && !completed; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }

  workers.reap();

  EXPECT_TRUE(completed);
  EXPECT_EQ(workers.size(), 0U);
}

TEST(ManagedWorkers, JoinsRunningThreadsAndRejectsNewWorkAfterStop)
{
  vixel_recorder::ManagedWorkers workers;
  std::atomic_bool release{false};
  std::atomic_bool completed{false};
  ASSERT_TRUE(workers.start([&release, &completed]() {
      while (!release) {std::this_thread::sleep_for(1ms);}
      completed = true;
    }));
  workers.request_stop();
  EXPECT_FALSE(workers.start([]() {}));
  release = true;

  workers.stop_and_join();

  EXPECT_TRUE(completed);
  EXPECT_EQ(workers.size(), 0U);
}
