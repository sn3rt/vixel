#pragma once

#include <cstdint>
#include <string>

namespace vixel_genicam
{

inline bool background_metering_allowed(
  bool capture_reserved, bool capture_primed)
{
  // Once an owner has reserved the camera and its initial metering frame has
  // completed, leave the pipeline empty so a background action cannot race the
  // owner's first requested exposure. This also covers one-shot sessions whose
  // requested interval is zero.
  return !capture_reserved || !capture_primed;
}

inline bool capture_preparation_ready(
  bool preparation_latched, bool capture_primed, bool pipeline_idle,
  bool trigger_armed_required, bool trigger_armed)
{
  return preparation_latched || (capture_primed && pipeline_idle &&
         (!trigger_armed_required || trigger_armed));
}

inline std::string capture_metering_domain(
  const std::string & sensor_id, const std::string & sync_group)
{
  return sync_group.empty() ? "sensor:" + sensor_id : sync_group;
}

inline std::string capture_status_detail(
  const std::string & operating_mode, bool ptp_action, bool ptp_locked,
  bool ptp_offset_readable, std::int64_t ptp_offset_ns, bool software_trigger)
{
  if (operating_mode != "capture") {return {};}
  if (ptp_action) {
    if (!ptp_locked) {return "Waiting for PTP synchronization";}
    auto detail = std::string("PTP scheduled capture ready");
    if (ptp_offset_readable) {
      detail += "; offset " + std::to_string(ptp_offset_ns) + " ns";
    }
    return detail;
  }
  if (software_trigger) {
    return "Software-triggered capture; exposure not synchronized";
  }
  return "Capture unavailable; free-running preview only";
}

}  // namespace vixel_genicam
