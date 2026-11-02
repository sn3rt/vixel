#pragma once

#include <string>

namespace vixel_genicam
{

inline bool session_endpoint_matches(
  bool record_present, const std::string & current_address,
  const std::string & current_interface, const std::string & assigned_address,
  const std::string & assigned_interface)
{
  if (!record_present) {return false;}
  if (!assigned_address.empty() && current_address != assigned_address) {return false;}
  if (!current_interface.empty() && current_interface != assigned_interface) {return false;}
  return true;
}

}  // namespace vixel_genicam
