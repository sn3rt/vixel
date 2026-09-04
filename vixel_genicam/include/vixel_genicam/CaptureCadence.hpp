#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace vixel_genicam
{

struct CaptureCadence
{
  std::uint32_t requested_interval_ms{0};
  double maximum_frame_rate_hz{0.0};
  double exposure_budget_us{0.0};
  bool trigger_overlap{false};
  std::uint32_t action_queue_size{1};
  std::uint32_t safety_margin_ms{10};
};

inline bool disabled_feature_value(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value == "off" || value == "false" || value == "disabled" ||
         value == "manual" || value == "no";
}

inline bool enabled_feature_value(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value == "on" || value == "true" || value == "enabled" || value == "yes";
}

inline std::optional<double> bounded_numeric_limit(
  double requested, double minimum, double maximum)
{
  if (!std::isfinite(requested) || !std::isfinite(minimum) ||
    !std::isfinite(maximum) || minimum > maximum || minimum > requested + 1.0)
  {
    return std::nullopt;
  }
  return std::clamp(requested, minimum, maximum);
}

inline std::uint32_t minimum_capture_interval_ms(const CaptureCadence & cadence)
{
  if (cadence.maximum_frame_rate_hz <= 0.0 ||
    !std::isfinite(cadence.maximum_frame_rate_hz))
  {
    return 0;
  }
  const double frame_period_ms = 1000.0 / cadence.maximum_frame_rate_hz;
  const double exposure_ms = std::max(0.0, cadence.exposure_budget_us / 1000.0);
  const double active_ms = cadence.trigger_overlap ?
    std::max(frame_period_ms, exposure_ms) : frame_period_ms + exposure_ms;
  // The exposure ceiling is derived from the same floating-point frame period.
  // Remove a sub-nanosecond rounding residue so an exactly negotiated 200 ms
  // cadence is not reported as 201 ms.
  return static_cast<std::uint32_t>(
    std::ceil(active_ms + cadence.safety_margin_ms - 1e-9));
}

inline std::optional<double> capture_exposure_limit_us(const CaptureCadence & cadence)
{
  if (cadence.requested_interval_ms == 0 || cadence.maximum_frame_rate_hz <= 0.0 ||
    !std::isfinite(cadence.maximum_frame_rate_hz))
  {
    return std::nullopt;
  }
  const auto period_us = static_cast<double>(cadence.requested_interval_ms) * 1000.0;
  const auto margin_us = static_cast<double>(cadence.safety_margin_ms) * 1000.0;
  const auto readout_us = 1000000.0 / cadence.maximum_frame_rate_hz;
  const auto available_us = cadence.trigger_overlap ?
    period_us - margin_us : period_us - readout_us - margin_us;
  if (available_us <= 0.0 || readout_us + margin_us > period_us) {
    return std::nullopt;
  }
  return available_us;
}

inline std::string capture_cadence_reason(const CaptureCadence & cadence)
{
  if (cadence.maximum_frame_rate_hz <= 0.0 ||
    !std::isfinite(cadence.maximum_frame_rate_hz))
  {
    return "camera maximum acquisition frame rate is unavailable";
  }
  const auto frame_period_ms = 1000.0 / cadence.maximum_frame_rate_hz;
  const auto exposure_ms = std::max(0.0, cadence.exposure_budget_us / 1000.0);
  const auto limiting_ms = cadence.trigger_overlap ?
    std::max(frame_period_ms, exposure_ms) : frame_period_ms + exposure_ms;
  const char * limit = cadence.trigger_overlap ?
    (exposure_ms >= frame_period_ms ? "exposure budget" : "frame period") :
    "exposure plus non-overlapped readout";
  return std::string(limit) + " requires " +
         std::to_string(static_cast<std::uint32_t>(std::ceil(limiting_ms))) +
         " ms plus " + std::to_string(cadence.safety_margin_ms) + " ms margin";
}

}  // namespace vixel_genicam
