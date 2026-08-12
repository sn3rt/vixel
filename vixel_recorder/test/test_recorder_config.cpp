#include "vixel_recorder/RecorderConfig.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

TEST(RecorderConfig, LoadsRecordingWithoutOptionalGpsSection)
{
  const auto path = std::filesystem::temp_directory_path() /
    "vixel-recorder-config-without-gps.yaml";
  std::ofstream(path) << R"(schema_version: 1
recording:
  root_directory: /data/vixel/captures
  minimum_free_bytes: 5368709120
)";

  const auto config = vixel_recorder::load_recorder_config(path.string());
  EXPECT_EQ(config.root_directory, "/data/vixel/captures");
  EXPECT_EQ(config.minimum_free_bytes, 5368709120ULL);
  EXPECT_FALSE(config.gps_enabled);
  EXPECT_EQ(config.gps_topic, "/fix");
  std::filesystem::remove(path);
}

TEST(RecorderConfig, UsesDefaultsWithoutRecordingSection)
{
  const auto path = std::filesystem::temp_directory_path() /
    "vixel-recorder-config-without-recording.yaml";
  std::ofstream(path) << "schema_version: 1\n";

  const auto config = vixel_recorder::load_recorder_config(path.string());
  EXPECT_EQ(config.root_directory, "/var/lib/vixel/captures");
  EXPECT_FALSE(config.gps_enabled);
  std::filesystem::remove(path);
}
