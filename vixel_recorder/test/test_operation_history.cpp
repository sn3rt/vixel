#include "vixel_recorder/OperationHistory.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace
{
struct Operation
{
  std::vector<std::string> capture_ids;
  std::uint64_t capture_id_count{0};
  bool capture_ids_truncated{false};
};
}  // namespace

TEST(OperationHistory, KeepsRecentIdsAndTracksTheFullCount)
{
  Operation operation;
  for (int index = 1; index <= 101; ++index) {
    vixel_recorder::append_bounded_capture_id(
      operation, "capture_" + std::to_string(index), 100);
  }

  ASSERT_EQ(operation.capture_ids.size(), 100U);
  EXPECT_EQ(operation.capture_ids.front(), "capture_2");
  EXPECT_EQ(operation.capture_ids.back(), "capture_101");
  EXPECT_EQ(operation.capture_id_count, 101U);
  EXPECT_TRUE(operation.capture_ids_truncated);
}

TEST(OperationHistory, RecognizesOnlyTerminalStatuses)
{
  EXPECT_TRUE(vixel_recorder::terminal_operation_status("complete"));
  EXPECT_TRUE(vixel_recorder::terminal_operation_status("failed"));
  EXPECT_TRUE(vixel_recorder::terminal_operation_status("cancelled"));
  EXPECT_FALSE(vixel_recorder::terminal_operation_status("running"));
  EXPECT_FALSE(vixel_recorder::terminal_operation_status("draining"));
}
