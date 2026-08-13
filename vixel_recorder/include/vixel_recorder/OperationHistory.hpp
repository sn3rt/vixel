#pragma once

#include <cstddef>
#include <string>
#include <utility>

namespace vixel_recorder
{

inline bool terminal_operation_status(const std::string & status)
{
  return status == "complete" || status == "failed" || status == "cancelled";
}

template<typename Operation>
void append_bounded_capture_id(
  Operation & operation, std::string capture_id, std::size_t limit)
{
  ++operation.capture_id_count;
  operation.capture_ids.push_back(std::move(capture_id));
  if (operation.capture_ids.size() > limit) {
    operation.capture_ids.erase(
      operation.capture_ids.begin(),
      operation.capture_ids.begin() + (operation.capture_ids.size() - limit));
  }
  operation.capture_ids_truncated = operation.capture_id_count > operation.capture_ids.size();
}

}  // namespace vixel_recorder
