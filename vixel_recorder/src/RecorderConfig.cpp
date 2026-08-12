#include "vixel_recorder/RecorderConfig.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

using namespace std::chrono_literals;

namespace vixel_recorder
{
namespace
{

template<typename T>
T read_or(const YAML::Node & node, const char * key, T fallback)
{
  if (!node.IsDefined() || !node.IsMap()) {return fallback;}
  const auto value = node[key];
  return value.IsDefined() ? value.as<T>() : fallback;
}

}  // namespace

RecorderConfig load_recorder_config(const std::string & path)
{
  RecorderConfig result;
  const auto root = YAML::LoadFile(path);
  const auto recording = root["recording"];
  result.root_directory = read_or(
    recording, "root_directory", result.root_directory.string());
  result.minimum_free_bytes = read_or(
    recording, "minimum_free_bytes", result.minimum_free_bytes);
  result.capture_timeout = std::chrono::milliseconds(
    read_or(recording, "capture_timeout_ms", static_cast<int>(result.capture_timeout.count())));
  result.sequence_dispatch_lead = std::chrono::milliseconds(read_or(
      recording, "sequence_dispatch_lead_ms",
      static_cast<int>(result.sequence_dispatch_lead.count())));
  result.sequence_prepare_timeout = std::chrono::milliseconds(read_or(
      recording, "sequence_prepare_timeout_ms",
      static_cast<int>(result.sequence_prepare_timeout.count())));
  result.recent_limit = read_or(recording, "recent_limit", result.recent_limit);
  result.max_inflight_captures = read_or(
    recording, "max_inflight_captures", result.max_inflight_captures);

  YAML::Node gps(YAML::NodeType::Map);
  if (recording.IsDefined() && recording.IsMap()) {
    const auto configured_gps = recording["gps"];
    if (configured_gps.IsDefined()) {gps = configured_gps;}
  }
  result.gps_enabled = read_or(gps, "enabled", result.gps_enabled);
  result.gps_topic = read_or(gps, "topic", result.gps_topic);
  result.gps_max_age = std::chrono::milliseconds(
    read_or(gps, "max_age_ms", static_cast<int>(result.gps_max_age.count())));

  if (result.root_directory.empty()) {throw std::runtime_error("recording root is empty");}
  if (result.capture_timeout < 1s || result.capture_timeout > 60s) {
    throw std::runtime_error("recording capture timeout must be between 1 and 60 seconds");
  }
  if (result.sequence_dispatch_lead < 20ms || result.sequence_dispatch_lead > 5s) {
    throw std::runtime_error(
            "recording sequence_dispatch_lead_ms must be between 20 and 5000");
  }
  if (result.sequence_prepare_timeout < 1s || result.sequence_prepare_timeout > 5min) {
    throw std::runtime_error(
            "recording sequence_prepare_timeout_ms must be between 1000 and 300000");
  }
  if (result.max_inflight_captures == 0 || result.max_inflight_captures > 256) {
    throw std::runtime_error("recording max_inflight_captures must be between 1 and 256");
  }
  if (result.gps_enabled && (result.gps_topic.empty() || result.gps_max_age.count() < 0)) {
    throw std::runtime_error("recording GPS topic/max_age_ms is invalid");
  }
  return result;
}

}  // namespace vixel_recorder
