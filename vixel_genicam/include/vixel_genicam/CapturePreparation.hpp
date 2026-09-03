#pragma once

#include <cstdint>

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

}  // namespace vixel_genicam
