#pragma once

#include <cstdint>

namespace vixel_genicam
{

inline bool background_metering_allowed(
  std::uint32_t requested_capture_interval_ms, bool capture_primed)
{
  // A non-zero interval means a recorder sequence is being prepared. Once its
  // initial metering frame has completed, leave the camera pipeline empty so a
  // metering action cannot occupy the camera immediately before cycle one.
  return requested_capture_interval_ms == 0 || !capture_primed;
}

inline bool capture_preparation_ready(
  bool preparation_latched, bool capture_primed, bool pipeline_idle,
  bool trigger_armed_required, bool trigger_armed)
{
  return preparation_latched || (capture_primed && pipeline_idle &&
         (!trigger_armed_required || trigger_armed));
}

}  // namespace vixel_genicam
