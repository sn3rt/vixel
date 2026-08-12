#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace vixel_recorder
{

struct RecorderConfig
{
  std::filesystem::path root_directory{"/var/lib/vixel/captures"};
  std::uintmax_t minimum_free_bytes{5ULL * 1024ULL * 1024ULL * 1024ULL};
  std::chrono::milliseconds capture_timeout{10000};
  std::chrono::milliseconds sequence_dispatch_lead{150};
  std::chrono::milliseconds sequence_prepare_timeout{60000};
  std::size_t recent_limit{100};
  std::size_t max_inflight_captures{32};
  bool gps_enabled{false};
  std::string gps_topic{"/fix"};
  std::chrono::milliseconds gps_max_age{2000};
};

RecorderConfig load_recorder_config(const std::string & path);

}  // namespace vixel_recorder
