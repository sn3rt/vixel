#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace vixel_genicam
{

struct NetworkConfig
{
  std::string id;
  std::string interface;
  std::string host_cidr;
  std::string gateway{"0.0.0.0"};
  std::int64_t packet_size{9000};
  std::int64_t packet_delay{100000};
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
  double frame_rate_hz{10.0};
};

struct GenicamConfig
{
  std::map<std::string, NetworkConfig> networks;
  std::vector<std::string> vendor_allowlist;
  int discovery_period_ms{2000};
  int image_timeout_ms{1000};
  int buffer_count{16};
  int socket_buffer_bytes{33554432};
  int software_trigger_lead_time_ms{10};
  int ptp_action_lead_time_ms{100};
  int encode_queue_depth{8};
  int capture_png_compression{1};
  std::int64_t ptp_tolerance_ns{100000};
  std::uint32_t action_device_key{1};
  int preview_width{960};
  std::string preview_format{"jpeg"};
  int png_compression{1};
  int jpeg_quality{70};
  ImagingConfig imaging;
};

GenicamConfig load_genicam_config(const std::string & path);

}  // namespace vixel_genicam
