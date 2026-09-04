#pragma once

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <vector>

namespace vixel_genicam
{

inline std::string normalized_trigger_value(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  return value;
}

inline std::optional<std::string> advertised_trigger_value(
  const std::vector<std::string> & available, const std::string & requested)
{
  const auto normalized_requested = normalized_trigger_value(requested);
  const auto match = std::find_if(
    available.begin(), available.end(), [&normalized_requested](const auto & candidate) {
      return normalized_trigger_value(candidate) == normalized_requested;
    });
  return match == available.end() ? std::nullopt : std::optional<std::string>(*match);
}

inline std::vector<std::string> preferred_trigger_selectors(
  const std::vector<std::string> & available)
{
  std::vector<std::string> result;
  for (const auto & preferred : {"FrameStart", "ExposureStart", "AcquisitionStart"}) {
    const auto match = advertised_trigger_value(available, preferred);
    if (match) {result.push_back(*match);}
  }
  for (const auto & candidate : available) {
    if (!advertised_trigger_value(result, candidate)) {result.push_back(candidate);}
  }
  return result;
}

}  // namespace vixel_genicam
