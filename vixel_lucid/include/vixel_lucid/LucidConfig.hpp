#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace vixel_lucid
{

struct NetworkConfig
{
  std::string id;
  std::string interface;
  std::string interface_mac;
  std::string host_cidr;
  std::int64_t packet_size{9000};
  std::int64_t packet_delay{1000};
  int transfer_slot_ms{100};
};

struct ImagingConfig
{
  std::string pixel_format{"BGR8"};
  bool use_max_width{true};
  bool use_max_height{true};
  std::string exposure_auto{"Continuous"};
  double exposure_time_us{1000.0};
  std::string gain_auto{"Continuous"};
  double gain_db{0.0};
};

struct LucidConfig
{
  std::map<std::string, NetworkConfig> networks;
  std::vector<std::string> model_allowlist;
  int discovery_period_ms{2000};
  int image_timeout_ms{5000};
  int schedule_lead_ms{250};
  std::int64_t action_device_key{1};
  std::int64_t action_group_key{1};
  std::int64_t action_group_mask{1};
  int preview_width{960};
  int jpeg_quality{70};
  ImagingConfig imaging;
};

LucidConfig load_lucid_config(const std::string & path);

}  // namespace vixel_lucid
