#include <vixel_lucid/LucidConfig.hpp>

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace vixel_lucid
{
namespace
{
template<typename T>
T read_or(const YAML::Node & node, const char * key, const T & fallback)
{
  return node && node[key] ? node[key].as<T>() : fallback;
}
}  // namespace

LucidConfig load_lucid_config(const std::string & path)
{
  const auto root = YAML::LoadFile(path);
  if (read_or(root, "schema_version", 1) != 1) {
    throw std::runtime_error("unsupported machine schema_version");
  }
  LucidConfig config;
  const auto defaults = root["defaults"];
  config.preview_width = read_or(defaults, "preview_width", config.preview_width);
  config.jpeg_quality = read_or(defaults, "jpeg_quality", config.jpeg_quality);

  const auto provider = root["providers"]["lucid"];
  config.model_allowlist = read_or(
    provider, "model_allowlist", std::vector<std::string>{});
  config.discovery_period_ms = read_or(
    provider, "discovery_period_ms", config.discovery_period_ms);
  config.image_timeout_ms = read_or(provider, "image_timeout_ms", config.image_timeout_ms);
  config.schedule_lead_ms = read_or(provider, "schedule_lead_ms", config.schedule_lead_ms);
  config.action_device_key = read_or(provider, "action_device_key", config.action_device_key);
  config.action_group_key = read_or(provider, "action_group_key", config.action_group_key);
  config.action_group_mask = read_or(provider, "action_group_mask", config.action_group_mask);
  const auto imaging = provider["imaging"];
  config.imaging.pixel_format = read_or(imaging, "pixel_format", config.imaging.pixel_format);
  config.imaging.use_max_width = read_or(imaging, "use_max_width", config.imaging.use_max_width);
  config.imaging.use_max_height = read_or(imaging, "use_max_height", config.imaging.use_max_height);
  config.imaging.exposure_auto = read_or(imaging, "exposure_auto", config.imaging.exposure_auto);
  config.imaging.exposure_time_us = read_or(
    imaging, "exposure_time_us", config.imaging.exposure_time_us);
  config.imaging.gain_auto = read_or(imaging, "gain_auto", config.imaging.gain_auto);
  config.imaging.gain_db = read_or(imaging, "gain_db", config.imaging.gain_db);

  const auto networks = root["managed_networks"];
  if (!networks || !networks.IsMap()) {
    throw std::runtime_error("machine configuration has no managed networks");
  }
  for (const auto & item : networks) {
    NetworkConfig network;
    network.id = item.first.as<std::string>();
    network.interface = item.second["interface"].as<std::string>();
    network.interface_mac = read_or(item.second, "interface_mac", std::string{});
    network.host_cidr = item.second["host_cidr"].as<std::string>();
    network.packet_size = read_or(item.second, "packet_size", network.packet_size);
    network.packet_delay = read_or(item.second, "packet_delay", network.packet_delay);
    network.transfer_slot_ms = read_or(
      item.second, "transfer_slot_ms", network.transfer_slot_ms);
    if (network.interface.empty() || network.host_cidr.empty() || network.packet_size <= 0 ||
      network.packet_delay < 0 || network.transfer_slot_ms < 0)
    {
      throw std::runtime_error("invalid managed network " + network.id);
    }
    config.networks.emplace(network.id, network);
  }
  if (config.discovery_period_ms < 250 || config.image_timeout_ms <= 0 ||
    config.schedule_lead_ms <= 0 || config.preview_width <= 0 ||
    config.jpeg_quality < 1 || config.jpeg_quality > 100)
  {
    throw std::runtime_error("invalid LUCID timing or preview configuration");
  }
  return config;
}

}  // namespace vixel_lucid
