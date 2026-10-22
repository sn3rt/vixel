#pragma once

#include <algorithm>

namespace vixel_genicam
{

template<typename TimePoint, typename IsArmed, typename Now, typename SleepFor>
bool wait_for_trigger_armed(
  const TimePoint deadline, const typename TimePoint::duration poll_interval,
  IsArmed && is_armed, Now && now, SleepFor && sleep_for)
{
  while (!is_armed()) {
    const auto current = now();
    if (current >= deadline) {return false;}
    sleep_for(std::min(poll_interval, deadline - current));
  }
  return true;
}

}  // namespace vixel_genicam
