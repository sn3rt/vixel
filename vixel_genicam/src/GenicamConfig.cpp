#include "vixel_genicam/GenicamConfig.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace vixel_genicam
{
namespace
{
template<typename T>
T read_or(const YAML::Node & node, const char * key, T fallback)
{
  return node && node[key] ? node[key].as<T>() : fallback;
}
}  // namespace

GenicamConfig load_genicam_config(const std::string & path)
{
  GenicamConfig config;
  const auto root = YAML::LoadFile(path);
  const auto defaults = root["defaults"];
  config.preview_width = read_or(defaults, "preview_width", config.preview_width);
  config.preview_format = read_or(defaults, "preview_format", config.preview_format);
  std::transform(
    config.preview_format.begin(), config.preview_format.end(), config.preview_format.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (config.preview_format == "jpg") {
    config.preview_format = "jpeg";
  }
  if (config.preview_format != "png" && config.preview_format != "jpeg") {
    throw std::runtime_error(
            "defaults.preview_format must be either 'png' or 'jpeg'");
  }
  config.png_compression = std::clamp(
    read_or(defaults, "png_compression", config.png_compression), 0, 9);
  config.jpeg_quality = read_or(defaults, "jpeg_quality", config.jpeg_quality);
  config.jpeg_quality = std::clamp(config.jpeg_quality, 1, 100);

  for (const auto & item : root["managed_networks"]) {
    NetworkConfig network;
    network.id = item.first.as<std::string>();
    const auto value = item.second;
    network.interface = value["interface"].as<std::string>();
    network.host_cidr = value["host_cidr"].as<std::string>();
    network.gateway = read_or(value, "gateway", network.gateway);
    network.packet_size = read_or(value, "packet_size", network.packet_size);
    network.packet_delay = read_or(value, "packet_delay", network.packet_delay);
    config.networks[network.id] = network;
  }

  const auto provider = root["providers"]["genicam"];
  config.vendor_allowlist = read_or(
    provider, "vendor_allowlist", std::vector<std::string>{});
  config.discovery_period_ms = read_or(
    provider, "discovery_period_ms", config.discovery_period_ms);
  config.image_timeout_ms = read_or(provider, "image_timeout_ms", config.image_timeout_ms);
  config.buffer_count = std::max(4, read_or(provider, "buffer_count", config.buffer_count));
  config.socket_buffer_bytes = read_or(
    provider, "socket_buffer_bytes", config.socket_buffer_bytes);
  const auto imaging = provider["imaging"];
  config.imaging.pixel_format = read_or(imaging, "pixel_format", config.imaging.pixel_format);
  config.imaging.use_max_width = read_or(
    imaging, "use_max_width", config.imaging.use_max_width);
  config.imaging.use_max_height = read_or(
    imaging, "use_max_height", config.imaging.use_max_height);
  config.imaging.exposure_auto = read_or(
    imaging, "exposure_auto", config.imaging.exposure_auto);
  config.imaging.exposure_time_us = read_or(
    imaging, "exposure_time_us", config.imaging.exposure_time_us);
  config.imaging.gain_auto = read_or(imaging, "gain_auto", config.imaging.gain_auto);
  config.imaging.gain_db = read_or(imaging, "gain_db", config.imaging.gain_db);
  config.imaging.frame_rate_hz = read_or(
    imaging, "frame_rate_hz", config.imaging.frame_rate_hz);
  return config;
}

}  // namespace vixel_genicam
