#include "vixel_recorder/RecorderConfig.hpp"
#include "vixel_recorder/OperationHistory.hpp"
#include "vixel_recorder/SequenceTiming.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <vixel_interfaces/action/record_capture.hpp>
#include <vixel_interfaces/msg/capture_frame_chunk.hpp>
#include <vixel_interfaces/msg/capture_operation.hpp>
#include <vixel_interfaces/msg/capture_operation_array.hpp>
#include <vixel_interfaces/msg/capture_record.hpp>
#include <vixel_interfaces/msg/capture_record_array.hpp>
#include <vixel_interfaces/msg/sensor_array.hpp>
#include <vixel_interfaces/msg/sync_group_array.hpp>
#include <vixel_interfaces/srv/capture_group.hpp>
#include <vixel_interfaces/srv/cancel_capture_operation.hpp>
#include <vixel_interfaces/srv/get_capture_operation.hpp>
#include <vixel_interfaces/srv/prepare_capture_groups.hpp>
#include <vixel_interfaces/srv/start_capture_sequence.hpp>
#include <vixel_interfaces/srv/submit_capture_batch.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace vixel_recorder
{
namespace
{
using CaptureGroup = vixel_interfaces::srv::CaptureGroup;
using CaptureRecord = vixel_interfaces::msg::CaptureRecord;
using CaptureOperation = vixel_interfaces::msg::CaptureOperation;
using PrepareCaptureGroups = vixel_interfaces::srv::PrepareCaptureGroups;
using RecordCapture = vixel_interfaces::action::RecordCapture;
using GoalHandle = rclcpp_action::ServerGoalHandle<RecordCapture>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<RecordCapture>;

struct OperationState
{
  CaptureOperation value;
  std::string request_prefix;
  bool stop_scheduling{false};
  bool cancelled_by_user{false};
  bool scheduling_done{false};
  bool terminal_recorded{false};
  std::map<std::uint32_t, std::size_t> cycle_results;
  std::map<std::uint32_t, std::size_t> cycle_failures;
};

std::string utc_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm value{};
  gmtime_r(&time, &value);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  std::ostringstream result;
  result << std::put_time(&value, "%Y-%m-%dT%H:%M:%S") << '.' <<
    std::setfill('0') << std::setw(3) << milliseconds.count() << 'Z';
  return result.str();
}

std::string generated_capture_id(std::uint64_t sequence)
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm value{};
  gmtime_r(&time, &value);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  std::ostringstream result;
  result << "capture_" << std::put_time(&value, "%Y%m%dT%H%M%S") <<
    std::setfill('0') << std::setw(3) << milliseconds.count() << "Z_" << sequence;
  return result.str();
}

builtin_interfaces::msg::Time system_time_message(
  const std::chrono::system_clock::time_point & value)
{
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    value.time_since_epoch()).count();
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(ns / 1000000000LL);
  result.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
  return result;
}

bool safe_identifier(const std::string & value)
{
  return !value.empty() && value.size() <= 128 &&
         std::all_of(value.begin(), value.end(), [](unsigned char character) {
           return std::isalnum(character) || character == '_' || character == '-' ||
                  character == '.';
         });
}

bool same_stamp(
  const builtin_interfaces::msg::Time & left, const builtin_interfaces::msg::Time & right)
{
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

std::pair<std::uint32_t, std::uint32_t> png_dimensions(
  const std::vector<std::uint8_t> & data)
{
  constexpr std::uint8_t signature[] = {137, 80, 78, 71, 13, 10, 26, 10};
  if (data.size() < 24 || !std::equal(std::begin(signature), std::end(signature), data.begin()) ||
    std::string(data.begin() + 12, data.begin() + 16) != "IHDR")
  {
    throw std::runtime_error("capture transport contains an invalid PNG header");
  }
  const auto read_u32 = [&data](std::size_t offset) {
      return (static_cast<std::uint32_t>(data[offset]) << 24U) |
             (static_cast<std::uint32_t>(data[offset + 1]) << 16U) |
             (static_cast<std::uint32_t>(data[offset + 2]) << 8U) |
             static_cast<std::uint32_t>(data[offset + 3]);
    };
  const auto width = read_u32(16);
  const auto height = read_u32(20);
  if (width == 0 || height == 0) {
    throw std::runtime_error("capture transport contains invalid PNG dimensions");
  }
  return {width, height};
}

nlohmann::json stamp_json(const builtin_interfaces::msg::Time & stamp)
{
  return {{"sec", stamp.sec}, {"nanosec", stamp.nanosec}};
}

nlohmann::json camera_info_json(const sensor_msgs::msg::CameraInfo & info)
{
  return {
    {"width", info.width}, {"height", info.height},
    {"distortion_model", info.distortion_model}, {"d", info.d},
    {"k", info.k}, {"r", info.r}, {"p", info.p},
    {"binning_x", info.binning_x}, {"binning_y", info.binning_y}
  };
}

struct FrameBucket
{
  struct ChunkAssembly
  {
    std_msgs::msg::Header header;
    std::string format;
    std::uint32_t chunk_count{0};
    std::size_t received_count{0};
    std::vector<std::vector<std::uint8_t>> chunks;
    std::vector<bool> received;
  };

  std::mutex mutex;
  std::condition_variable changed;
  std::map<
    std::string,
    std::map<std::string, sensor_msgs::msg::CompressedImage::ConstSharedPtr>> images;
  std::map<std::string, std::map<std::string, ChunkAssembly>> assemblies;
  std::map<std::string, std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr>> camera_info;
};

struct CaptureSubscriptions
{
  std::shared_ptr<FrameBucket> frames{std::make_shared<FrameBucket>()};
  std::map<
    std::string,
    rclcpp::Subscription<vixel_interfaces::msg::CaptureFrameChunk>::SharedPtr>
  image_subscriptions;
  std::map<
    std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>
  info_subscriptions;
};

void append_bounded(
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> & values,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr value)
{
  values.push_back(std::move(value));
  if (values.size() > 4) {values.erase(values.begin());}
}

void accept_chunk(
  FrameBucket & frames, const std::string & sensor_id,
  const vixel_interfaces::msg::CaptureFrameChunk & chunk)
{
  if (chunk.chunk_count == 0 || chunk.chunk_index >= chunk.chunk_count ||
    chunk.capture_id.empty() || chunk.sensor_id != sensor_id)
  {
    return;
  }
  auto & sensor_assemblies = frames.assemblies[sensor_id];
  if (sensor_assemblies.count(chunk.capture_id) == 0) {
    while (sensor_assemblies.size() >= 64) {
      sensor_assemblies.erase(sensor_assemblies.begin());
    }
  }
  auto & assembly = sensor_assemblies[chunk.capture_id];
  if (assembly.chunk_count == 0) {
    assembly.header = chunk.header;
    assembly.format = chunk.format;
    assembly.chunk_count = chunk.chunk_count;
    assembly.chunks.resize(chunk.chunk_count);
    assembly.received.resize(chunk.chunk_count, false);
  }
  if (assembly.chunk_count != chunk.chunk_count ||
    !same_stamp(assembly.header.stamp, chunk.header.stamp))
  {
    frames.assemblies[sensor_id].erase(chunk.capture_id);
    return;
  }
  if (!assembly.received[chunk.chunk_index]) {
    assembly.chunks[chunk.chunk_index].assign(chunk.data.begin(), chunk.data.end());
    assembly.received[chunk.chunk_index] = true;
    ++assembly.received_count;
  }
  if (assembly.received_count != assembly.chunk_count) {return;}
  auto image = std::make_shared<sensor_msgs::msg::CompressedImage>();
  image->header = assembly.header;
  image->format = assembly.format;
  const auto total = std::accumulate(
    assembly.chunks.begin(), assembly.chunks.end(), std::size_t{0},
    [](std::size_t size, const auto & value) {return size + value.size();});
  image->data.reserve(total);
  for (const auto & value : assembly.chunks) {
    image->data.insert(image->data.end(), value.begin(), value.end());
  }
  auto & sensor_images = frames.images[sensor_id];
  if (sensor_images.count(chunk.capture_id) == 0) {
    while (sensor_images.size() >= 64) {sensor_images.erase(sensor_images.begin());}
  }
  sensor_images[chunk.capture_id] = std::move(image);
  frames.assemblies[sensor_id].erase(chunk.capture_id);
  frames.changed.notify_all();
}

}  // namespace

class CaptureRecorder : public rclcpp::Node
{
public:
  CaptureRecorder()
  : Node("capture_recorder", "/vixel")
  {
    const auto machine_file = declare_parameter<std::string>(
      "machine_file", "/etc/vixel/machine.yaml");
    config_ = load_recorder_config(machine_file);
    std::filesystem::create_directories(config_.root_directory);
    scan_records();

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    sensors_subscription_ = create_subscription<vixel_interfaces::msg::SensorArray>(
      "/vixel/sensors", state_qos,
      [this](const vixel_interfaces::msg::SensorArray::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sensors_.clear();
        for (const auto & sensor : message->sensors) {
          sensors_[sensor.sensor_id] = sensor;
          ensure_capture_subscriptions_locked(sensor);
        }
      });
    groups_subscription_ = create_subscription<vixel_interfaces::msg::SyncGroupArray>(
      "/vixel/sync_groups", state_qos,
      [this](const vixel_interfaces::msg::SyncGroupArray::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        groups_.clear();
        for (const auto & group : message->groups) {groups_[group.group_id] = group;}
        state_changed_.notify_all();
      });
    if (config_.gps_enabled) {
      gps_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        config_.gps_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr fix) {
          std::lock_guard<std::mutex> lock(gps_mutex_);
          latest_gps_ = std::move(fix);
        });
    }
    capture_client_ = create_client<CaptureGroup>("/vixel/capture_group");
    prepare_capture_groups_client_ = create_client<PrepareCaptureGroups>(
      "/vixel/prepare_capture_groups");
    capture_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    records_publisher_ = create_publisher<vixel_interfaces::msg::CaptureRecordArray>(
      "/vixel/capture_records", state_qos);
    action_server_ = rclcpp_action::create_server<RecordCapture>(
      this, "/vixel/record_capture",
      std::bind(&CaptureRecorder::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CaptureRecorder::handle_cancel, this, std::placeholders::_1),
      std::bind(&CaptureRecorder::handle_accepted, this, std::placeholders::_1));
    operation_capture_client_ = rclcpp_action::create_client<RecordCapture>(
      this, "/vixel/record_capture");
    operations_publisher_ = create_publisher<vixel_interfaces::msg::CaptureOperationArray>(
      "/vixel/capture_operations", state_qos);
    submit_batch_service_ = create_service<vixel_interfaces::srv::SubmitCaptureBatch>(
      "/vixel/submit_capture_batch",
      std::bind(
        &CaptureRecorder::submit_capture_batch, this, std::placeholders::_1,
        std::placeholders::_2));
    start_sequence_service_ = create_service<vixel_interfaces::srv::StartCaptureSequence>(
      "/vixel/start_capture_sequence",
      std::bind(
        &CaptureRecorder::start_capture_sequence, this, std::placeholders::_1,
        std::placeholders::_2));
    get_operation_service_ = create_service<vixel_interfaces::srv::GetCaptureOperation>(
      "/vixel/get_capture_operation",
      std::bind(
        &CaptureRecorder::get_capture_operation, this, std::placeholders::_1,
        std::placeholders::_2));
    cancel_operation_service_ = create_service<vixel_interfaces::srv::CancelCaptureOperation>(
      "/vixel/cancel_capture_operation",
      std::bind(
        &CaptureRecorder::cancel_capture_operation, this, std::placeholders::_1,
        std::placeholders::_2));
    publish_records();
    publish_operations();
    RCLCPP_INFO(
      get_logger(), "Capture recorder ready at %s (minimum free %.2f GiB)",
      config_.root_directory.c_str(),
      static_cast<double>(config_.minimum_free_bytes) / (1024.0 * 1024.0 * 1024.0));
  }

private:
  nlohmann::json gps_for_capture(const builtin_interfaces::msg::Time & capture_time)
  {
    sensor_msgs::msg::NavSatFix::ConstSharedPtr fix;
    {
      std::lock_guard<std::mutex> lock(gps_mutex_);
      fix = latest_gps_;
    }
    if (!fix || fix->status.status < 0 || !std::isfinite(fix->latitude) ||
      !std::isfinite(fix->longitude) || !std::isfinite(fix->altitude))
    {
      return nlohmann::json::object();
    }
    const auto capture_ns = static_cast<std::int64_t>(capture_time.sec) * 1000000000LL +
      capture_time.nanosec;
    const auto fix_ns = static_cast<std::int64_t>(fix->header.stamp.sec) * 1000000000LL +
      fix->header.stamp.nanosec;
    const auto age_ms = std::llabs(capture_ns - fix_ns) / 1000000LL;
    if (fix_ns == 0 || age_ms > config_.gps_max_age.count()) {
      return nlohmann::json::object();
    }
    return {
      {"topic", config_.gps_topic}, {"stamp", stamp_json(fix->header.stamp)},
      {"age_ms", age_ms}, {"frame_id", fix->header.frame_id},
      {"status", fix->status.status}, {"service", fix->status.service},
      {"latitude", fix->latitude}, {"longitude", fix->longitude},
      {"altitude", fix->altitude},
      {"position_covariance", fix->position_covariance},
      {"position_covariance_type", fix->position_covariance_type}
    };
  }

  std::optional<std::string> validate_operation_request(
    const std::vector<std::string> & group_ids, const std::string & request_id,
    const std::string & metadata_json, bool require_capture_ready = true)
  {
    if (group_ids.empty()) {return "at least one synchronization group is required";}
    if (request_id.size() > 64 || (!request_id.empty() && !safe_identifier(request_id))) {
      return "request_id must be a safe identifier of at most 64 characters";
    }
    if (metadata_json.size() > 65536) {return "metadata_json exceeds 64 KiB";}
    if (!metadata_json.empty()) {
      try {
        if (!nlohmann::json::parse(metadata_json).is_object()) {
          return "metadata_json must contain a JSON object";
        }
      } catch (const std::exception & error) {
        return std::string("metadata_json is invalid: ") + error.what();
      }
    }
    std::set<std::string> unique;
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & group_id : group_ids) {
      if (!unique.insert(group_id).second) {return "group_ids contains a duplicate";}
      const auto group = groups_.find(group_id);
      if (group == groups_.end()) {return "unknown synchronization group " + group_id;}
      if (require_capture_ready &&
        (group->second.operating_mode != "capture" || !group->second.ready))
      {
        return "synchronization group " + group_id + " is not capture-ready";
      }
    }
    return std::nullopt;
  }

  std::optional<std::string> validate_sequence_interval(
    const std::vector<std::string> & group_ids, std::uint32_t interval_ms)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & group_id : group_ids) {
      const auto group = groups_.find(group_id);
      if (group == groups_.end() || group->second.minimum_capture_interval_ms == 0) {
        continue;
      }
      if (!capture_interval_supported(
          interval_ms, group->second.minimum_capture_interval_ms))
      {
        std::ostringstream message;
        message << "requested interval " << interval_ms << " ms is faster than group " <<
          group_id << " minimum " << group->second.minimum_capture_interval_ms << " ms";
        if (!group->second.cadence_limit_reason.empty()) {
          message << " (" << group->second.cadence_limit_reason << ')';
        }
        return message.str();
      }
    }
    return std::nullopt;
  }

  std::uint32_t operation_action_queue_size(const std::vector<std::string> & group_ids)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::uint32_t result = std::numeric_limits<std::uint32_t>::max();
    for (const auto & group_id : group_ids) {
      const auto group = groups_.find(group_id);
      const auto size = group == groups_.end() || group->second.action_queue_size == 0 ?
        1U : group->second.action_queue_size;
      result = std::min(result, size);
    }
    return result == std::numeric_limits<std::uint32_t>::max() ? 1U : result;
  }

  std::string make_operation_id()
  {
    auto value = generated_capture_id(++operation_sequence_);
    value.replace(0, std::string("capture").size(), "operation");
    return value;
  }

  std::optional<std::string> reserve_groups(
    const std::vector<std::string> & group_ids, const std::string & operation_id)
  {
    std::lock_guard<std::mutex> lock(group_reservations_mutex_);
    for (const auto & group_id : group_ids) {
      const auto reserved = group_reservations_.find(group_id);
      if (reserved != group_reservations_.end()) {
        return "synchronization group " + group_id +
               " is reserved by operation " + reserved->second;
      }
      const auto active = active_group_captures_.find(group_id);
      if (active != active_group_captures_.end() && active->second != 0) {
        return "synchronization group " + group_id + " already has an active capture";
      }
    }
    for (const auto & group_id : group_ids) {group_reservations_[group_id] = operation_id;}
    return std::nullopt;
  }

  std::optional<std::string> group_reservation_conflict(
    const std::vector<std::string> & group_ids)
  {
    std::lock_guard<std::mutex> lock(group_reservations_mutex_);
    for (const auto & group_id : group_ids) {
      const auto reserved = group_reservations_.find(group_id);
      if (reserved != group_reservations_.end()) {
        return "synchronization group " + group_id +
               " is reserved by operation " + reserved->second;
      }
      const auto active = active_group_captures_.find(group_id);
      if (active != active_group_captures_.end() && active->second != 0) {
        return "synchronization group " + group_id + " already has an active capture";
      }
    }
    return std::nullopt;
  }

  void release_groups(
    const std::vector<std::string> & group_ids, const std::string & operation_id)
  {
    std::lock_guard<std::mutex> lock(group_reservations_mutex_);
    for (const auto & group_id : group_ids) {
      const auto reserved = group_reservations_.find(group_id);
      if (reserved != group_reservations_.end() && reserved->second == operation_id) {
        group_reservations_.erase(reserved);
      }
    }
  }

  std::shared_ptr<OperationState> create_operation(
    const std::string & kind, const std::vector<std::string> & group_ids,
    const std::string & request_id, std::uint32_t count, std::uint32_t interval_ms,
    bool synchronize_groups, const std::string & metadata_json,
    const builtin_interfaces::msg::Time & first_time,
    const std::string & operation_id = {})
  {
    auto operation = std::make_shared<OperationState>();
    operation->value.stamp = now();
    operation->value.operation_id = operation_id.empty() ? make_operation_id() : operation_id;
    operation->value.kind = kind;
    operation->value.status = "accepted";
    operation->value.message = "capture operation accepted";
    operation->value.group_ids = group_ids;
    operation->value.requested_cycles = count;
    operation->value.interval_ms = interval_ms;
    operation->value.synchronize_groups = synchronize_groups;
    operation->value.metadata_json = metadata_json;
    operation->value.first_scheduled_time = first_time;
    operation->request_prefix = request_id.empty() ? operation->value.operation_id : request_id;
    {
      std::lock_guard<std::mutex> lock(operations_mutex_);
      const auto active_count = std::count_if(
        operations_.begin(), operations_.end(), [](const auto & item) {
          // A cancellation request may expose a terminal-looking status while
          // its preparation worker is still unwinding. Count it as active until
          // finish_operation_locked records the terminal transition.
          return !item.second->terminal_recorded;
        });
      if (active_count >= static_cast<std::ptrdiff_t>(config_.max_active_operations)) {
        return {};
      }
      operations_[operation->value.operation_id] = operation;
      ++operations_generation_;
    }
    publish_operations();
    return operation;
  }

  void submit_capture_batch(
    const std::shared_ptr<vixel_interfaces::srv::SubmitCaptureBatch::Request> request,
    std::shared_ptr<vixel_interfaces::srv::SubmitCaptureBatch::Response> response)
  {
    if (const auto error = validate_operation_request(
        request->group_ids, request->request_id, request->metadata_json))
    {
      response->accepted = false;
      response->message = *error;
      return;
    }
    if (const auto error = group_reservation_conflict(request->group_ids)) {
      response->accepted = false;
      response->message = *error;
      return;
    }
    const auto first = std::chrono::system_clock::now() + 750ms;
    const auto first_message = system_time_message(first);
    auto operation = create_operation(
      "batch", request->group_ids, request->request_id, 1, 0,
      request->synchronize_groups, request->metadata_json,
      request->synchronize_groups ? first_message : builtin_interfaces::msg::Time{});
    if (!operation) {
      response->accepted = false;
      response->message = "recorder reached the active operation limit";
      return;
    }
    response->accepted = true;
    response->message = "batch accepted for asynchronous capture";
    response->operation_id = operation->value.operation_id;
    if (request->synchronize_groups) {response->scheduled_time = first_message;}
    std::thread([this, operation, first]() {schedule_operation(operation, first, 1);}).detach();
  }

  void start_capture_sequence(
    const std::shared_ptr<vixel_interfaces::srv::StartCaptureSequence::Request> request,
    std::shared_ptr<vixel_interfaces::srv::StartCaptureSequence::Response> response)
  {
    if (const auto error = validate_operation_request(
        request->group_ids, request->request_id, request->metadata_json, false))
    {
      response->accepted = false;
      response->message = *error;
      return;
    }
    if (request->interval_ms < 100 || request->interval_ms > 86400000U) {
      response->accepted = false;
      response->message = "interval_ms must be between 100 and 86400000";
      return;
    }
    const auto operation_id = make_operation_id();
    if (const auto error = reserve_groups(request->group_ids, operation_id)) {
      response->accepted = false;
      response->message = *error;
      return;
    }
    auto operation = create_operation(
      "sequence", request->group_ids, request->request_id, request->count,
      request->interval_ms, request->synchronize_groups, request->metadata_json,
      builtin_interfaces::msg::Time{}, operation_id);
    if (!operation) {
      release_groups(request->group_ids, operation_id);
      response->accepted = false;
      response->message = "recorder reached the active operation limit";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(operations_mutex_);
      operation->value.status = "preparing";
      operation->value.message = "configuring capture groups for requested interval";
      operation->value.stamp = now();
      ++operations_generation_;
    }
    publish_operations();
    response->accepted = true;
    response->message = "sequence accepted; preparing capture cadence";
    response->operation_id = operation->value.operation_id;
    std::thread(
      [this, operation, count = request->count]() {
        prepare_and_schedule_sequence(operation, count);
      }).detach();
  }

  void fail_sequence_preparation(
    const std::shared_ptr<OperationState> & operation, const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(operations_mutex_);
      operation->stop_scheduling = true;
      operation->scheduling_done = true;
      operation->value.status = operation->cancelled_by_user ? "cancelled" : "failed";
      operation->value.message = message;
      operation->value.stamp = now();
      record_terminal_operation_locked(*operation);
      ++operations_generation_;
    }
    release_groups(operation->value.group_ids, operation->value.operation_id);
    publish_operations();
  }

  void prepare_and_schedule_sequence(
    const std::shared_ptr<OperationState> & operation, std::uint32_t count)
  {
    if (!prepare_capture_groups_client_->wait_for_service(2s)) {
      fail_sequence_preparation(operation, "capture group preparation service is unavailable");
      return;
    }
    auto request = std::make_shared<PrepareCaptureGroups::Request>();
    request->group_ids = operation->value.group_ids;
    request->interval_ms = operation->value.interval_ms;
    auto future = prepare_capture_groups_client_->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
      fail_sequence_preparation(operation, "timed out requesting capture group preparation");
      return;
    }
    const auto response = future.get();
    if (!response->accepted) {
      fail_sequence_preparation(operation, response->message);
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() + config_.sequence_prepare_timeout;
    std::string last_status{"waiting for capture group status"};
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      bool stopped = false;
      bool cancelled = false;
      std::string stopped_message;
      {
        std::lock_guard<std::mutex> lock(operations_mutex_);
        stopped = operation->stop_scheduling;
        cancelled = operation->cancelled_by_user;
        stopped_message = operation->value.message;
      }
      if (stopped) {
        fail_sequence_preparation(
          operation, cancelled ? "sequence cancelled during preparation" : stopped_message);
        return;
      }

      bool ready = true;
      std::optional<std::string> permanent_error;
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        for (const auto & group_id : operation->value.group_ids) {
          const auto iterator = groups_.find(group_id);
          if (iterator == groups_.end()) {
            ready = false;
            last_status = "waiting for synchronization group " + group_id;
            continue;
          }
          const auto & group = iterator->second;
          if (group.operating_mode != "capture" ||
            group.requested_capture_interval_ms != operation->value.interval_ms)
          {
            ready = false;
            last_status = "waiting for " + group_id + " to apply " +
              std::to_string(operation->value.interval_ms) + " ms cadence";
            continue;
          }
          if (!group.cadence_configured) {
            ready = false;
            last_status = group.last_error.empty() ?
              "waiting for " + group_id + " cadence configuration" : group.last_error;
            continue;
          }
          if (!group.cadence_ready) {
            permanent_error = group.cadence_limit_reason.empty() ?
              (group.last_error.empty() ?
              "synchronization group " + group_id +
              " cannot satisfy the requested capture interval" : group.last_error) :
              group.cadence_limit_reason;
            break;
          }
          if (!group.ready) {
            ready = false;
            last_status = group.last_error.empty() ?
              "waiting for synchronization group " + group_id + " readiness" :
              group.last_error;
          }
        }
        if (!ready && !permanent_error) {state_changed_.wait_for(lock, 100ms);}
      }
      if (permanent_error) {
        fail_sequence_preparation(operation, *permanent_error);
        return;
      }
      if (ready) {
        if (const auto error = validate_sequence_interval(
            operation->value.group_ids, operation->value.interval_ms))
        {
          fail_sequence_preparation(operation, *error);
          return;
        }
        const auto first = std::chrono::system_clock::now() + 1000ms;
        {
          std::lock_guard<std::mutex> lock(operations_mutex_);
          operation->value.first_scheduled_time = system_time_message(first);
          operation->value.message = "capture groups prepared; scheduling captures";
          operation->value.stamp = now();
          ++operations_generation_;
        }
        publish_operations();
        schedule_operation(operation, first, count);
        return;
      }
    }
    fail_sequence_preparation(
      operation, "capture group preparation timed out: " + last_status);
  }

  void get_capture_operation(
    const std::shared_ptr<vixel_interfaces::srv::GetCaptureOperation::Request> request,
    std::shared_ptr<vixel_interfaces::srv::GetCaptureOperation::Response> response)
  {
    std::lock_guard<std::mutex> lock(operations_mutex_);
    const auto operation = operations_.find(request->operation_id);
    if (operation == operations_.end()) {
      response->success = false;
      response->message = "unknown capture operation";
      return;
    }
    response->success = true;
    response->message = "capture operation found";
    response->operation = operation->second->value;
  }

  void cancel_capture_operation(
    const std::shared_ptr<vixel_interfaces::srv::CancelCaptureOperation::Request> request,
    std::shared_ptr<vixel_interfaces::srv::CancelCaptureOperation::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(operations_mutex_);
      const auto operation = operations_.find(request->operation_id);
      if (operation == operations_.end()) {
        response->success = false;
        response->message = "unknown capture operation";
        return;
      }
      auto & state = *operation->second;
      if (state.scheduling_done && state.value.pending_saves == 0) {
        response->success = false;
        response->message = "capture operation has already finished";
        response->operation = state.value;
        return;
      }
      state.stop_scheduling = true;
      state.cancelled_by_user = true;
      state.value.status = state.value.pending_saves == 0 ? "cancelled" : "draining";
      state.value.message = "future captures cancelled; accepted captures will finish";
      state.value.stamp = now();
      ++operations_generation_;
      response->success = true;
      response->message = state.value.message;
      response->operation = state.value;
    }
    publish_operations();
  }

  void schedule_operation(
    const std::shared_ptr<OperationState> & operation,
    const std::chrono::system_clock::time_point & first_time, std::uint32_t count)
  {
    if (!operation_capture_client_->wait_for_action_server(2s)) {
      {
        std::lock_guard<std::mutex> lock(operations_mutex_);
        operation->stop_scheduling = true;
        operation->scheduling_done = true;
        operation->value.status = "failed";
        operation->value.message = "capture action server is unavailable";
        operation->value.stamp = now();
        record_terminal_operation_locked(*operation);
        ++operations_generation_;
      }
      if (operation->value.kind == "sequence") {
        release_groups(operation->value.group_ids, operation->value.operation_id);
      }
      publish_operations();
      return;
    }
    const auto interval = std::chrono::milliseconds(operation->value.interval_ms);
    const auto action_queue_size = operation_action_queue_size(operation->value.group_ids);
    const auto dispatch_lead = operation->value.kind == "sequence" ?
      sequence_dispatch_lead(config_.sequence_dispatch_lead, interval, action_queue_size) :
      config_.sequence_dispatch_lead;
    std::uint32_t cycle = 1;
    while (rclcpp::ok() && (count == 0 || cycle <= count)) {
      const auto target = first_time + interval * (cycle - 1);
      const auto dispatch = target - dispatch_lead;
      while (rclcpp::ok() && std::chrono::system_clock::now() < dispatch) {
        {
          std::lock_guard<std::mutex> lock(operations_mutex_);
          if (operation->stop_scheduling) {break;}
        }
        std::this_thread::sleep_for(20ms);
      }
      {
        std::lock_guard<std::mutex> lock(operations_mutex_);
        if (operation->stop_scheduling) {break;}
      }
      {
        std::lock_guard<std::mutex> lock(active_captures_mutex_);
        if (active_capture_count_ + operation->value.group_ids.size() >
          config_.max_inflight_captures)
        {
          std::lock_guard<std::mutex> operations_lock(operations_mutex_);
          operation->stop_scheduling = true;
          operation->value.status = "draining";
          operation->value.message =
            "sequence stopped at recorder capacity; accepted captures will finish";
          ++operations_generation_;
          break;
        }
      }
      dispatch_cycle(operation, cycle, target);
      ++cycle;
    }
    bool release_reservation = false;
    {
      std::lock_guard<std::mutex> lock(operations_mutex_);
      operation->scheduling_done = true;
      if (operation->value.pending_saves == 0) {
        finish_operation_locked(*operation);
        release_reservation = operation->value.kind == "sequence";
      }
      else if (!operation->stop_scheduling) {
        operation->value.status = "draining";
        operation->value.message = "all captures scheduled; waiting for saves";
      }
      operation->value.stamp = now();
      ++operations_generation_;
    }
    if (release_reservation) {
      release_groups(operation->value.group_ids, operation->value.operation_id);
    }
    publish_operations();
  }

  void dispatch_cycle(
    const std::shared_ptr<OperationState> & operation, std::uint32_t cycle,
    const std::chrono::system_clock::time_point & target)
  {
    const auto requested_time = system_time_message(target);
    std::vector<std::pair<std::string, RecordCapture::Goal>> goals;
    {
      std::lock_guard<std::mutex> lock(operations_mutex_);
      operation->value.status = "running";
      operation->value.message = "capturing";
      std::size_t group_index = 0;
      bool identifiers_valid = true;
      for (const auto & group_id : operation->value.group_ids) {
        RecordCapture::Goal goal;
        goal.group_id = group_id;
        goal.request_id = operation->request_prefix + "_" + std::to_string(cycle) + "_" + group_id;
        if (!safe_identifier(goal.request_id)) {
          identifiers_valid = false;
          break;
        }
        goal.has_requested_time = operation->value.kind == "sequence" ||
          operation->value.synchronize_groups;
        const auto group_target = target +
          (operation->value.synchronize_groups ? 0ms :
          std::chrono::milliseconds(5 * static_cast<std::int64_t>(group_index)));
        goal.requested_time = system_time_message(group_target);
        goal.operation_id = operation->value.operation_id;
        goal.cycle = cycle;
        goal.metadata_json = operation->value.metadata_json;
        goals.emplace_back(group_id, std::move(goal));
        ++group_index;
      }
      if (!identifiers_valid) {
        goals.clear();
        operation->stop_scheduling = true;
        operation->value.status = "failed";
        operation->value.message = "generated capture ID is too long or unsafe";
      } else {
        for (const auto & item : goals) {
          append_bounded_capture_id(
            operation->value, item.second.request_id, config_.operation_capture_id_limit);
          ++operation->value.pending_saves;
        }
        ++operation->value.scheduled_cycles;
        if (operation->value.kind == "sequence" || operation->value.synchronize_groups) {
          operation->value.last_scheduled_time = requested_time;
        }
      }
      operation->value.stamp = now();
      ++operations_generation_;
    }
    publish_operations();
    if (operation->stop_scheduling) {return;}
    for (auto & item : goals) {
      rclcpp_action::Client<RecordCapture>::SendGoalOptions options;
      options.goal_response_callback =
        [this, operation, cycle](const ClientGoalHandle::SharedPtr & handle) {
          if (!handle) {
            capture_finished(operation, cycle, false, "capture action rejected");
          }
        };
      options.result_callback =
        [this, operation, cycle](const ClientGoalHandle::WrappedResult & result) {
          const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success;
          const auto message = result.result ? result.result->message : "capture action failed";
          capture_finished(operation, cycle, success, message);
        };
      try {
        operation_capture_client_->async_send_goal(item.second, options);
      } catch (const std::exception & error) {
        capture_finished(operation, cycle, false, error.what());
      }
    }
  }

  void capture_finished(
    const std::shared_ptr<OperationState> & operation, std::uint32_t cycle,
    bool success, const std::string & message)
  {
    bool release_reservation = false;
    {
      std::lock_guard<std::mutex> lock(operations_mutex_);
      if (operation->value.pending_saves != 0) {--operation->value.pending_saves;}
      ++operation->cycle_results[cycle];
      if (!success) {
        ++operation->cycle_failures[cycle];
        operation->stop_scheduling = true;
        operation->value.message = message;
      }
      if (operation->cycle_results[cycle] == operation->value.group_ids.size()) {
        if (operation->cycle_failures[cycle] == 0) {++operation->value.completed_cycles;}
        else {++operation->value.failed_cycles;}
        operation->cycle_results.erase(cycle);
        operation->cycle_failures.erase(cycle);
      }
      if (operation->scheduling_done && operation->value.pending_saves == 0) {
        finish_operation_locked(*operation);
        release_reservation = operation->value.kind == "sequence";
      }
      operation->value.stamp = now();
      ++operations_generation_;
    }
    if (release_reservation) {
      release_groups(operation->value.group_ids, operation->value.operation_id);
    }
    publish_operations();
  }

  void finish_operation_locked(OperationState & operation)
  {
    if (operation.stop_scheduling) {
      operation.value.status = operation.cancelled_by_user ? "cancelled" : "failed";
      if (operation.value.message.empty()) {
        operation.value.message = "capture operation stopped";
      }
    } else if (operation.value.failed_cycles != 0) {
      operation.value.status = "failed";
      operation.value.message = "one or more capture cycles failed";
    } else {
      operation.value.status = "complete";
      operation.value.message = "all capture cycles saved";
    }
    record_terminal_operation_locked(operation);
  }

  void record_terminal_operation_locked(OperationState & operation)
  {
    if (operation.terminal_recorded || !terminal_operation_status(operation.value.status)) {
      return;
    }
    operation.terminal_recorded = true;
    terminal_operation_ids_.push_back(operation.value.operation_id);
    while (terminal_operation_ids_.size() > config_.operation_history_limit) {
      const auto expired = terminal_operation_ids_.front();
      terminal_operation_ids_.pop_front();
      const auto iterator = operations_.find(expired);
      if (iterator != operations_.end() &&
        terminal_operation_status(iterator->second->value.status))
      {
        operations_.erase(iterator);
      }
    }
  }

  void publish_operations()
  {
    if (!operations_publisher_) {return;}
    vixel_interfaces::msg::CaptureOperationArray message;
    message.header.stamp = now();
    {
      std::lock_guard<std::mutex> lock(operations_mutex_);
      message.generation = operations_generation_;
      for (auto iterator = operations_.rbegin(); iterator != operations_.rend(); ++iterator) {
        message.operations.push_back(iterator->second->value);
      }
    }
    operations_publisher_->publish(message);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const RecordCapture::Goal> goal)
  {
    if (goal->group_id.empty()) {return rclcpp_action::GoalResponse::REJECT;}
    if (!goal->request_id.empty() && !safe_identifier(goal->request_id)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->metadata_json.size() > 65536) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal->metadata_json.empty()) {
      try {
        if (!nlohmann::json::parse(goal->metadata_json).is_object()) {
          return rclcpp_action::GoalResponse::REJECT;
        }
      } catch (...) {
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto group = groups_.find(goal->group_id);
      if (group == groups_.end() || group->second.member_ids.empty()) {
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    {
      std::lock_guard<std::mutex> lock(group_reservations_mutex_);
      const auto reserved = group_reservations_.find(goal->group_id);
      if (reserved != group_reservations_.end() &&
        (goal->operation_id.empty() || goal->operation_id != reserved->second))
      {
        return rclcpp_action::GoalResponse::REJECT;
      }
      const auto active = active_group_captures_.find(goal->group_id);
      if (reserved == group_reservations_.end() &&
        active != active_group_captures_.end() && active->second != 0)
      {
        return rclcpp_action::GoalResponse::REJECT;
      }
      ++active_group_captures_[goal->group_id];
    }
    {
      std::lock_guard<std::mutex> lock(active_captures_mutex_);
      if (active_capture_count_ >= config_.max_inflight_captures ||
        (!goal->request_id.empty() && active_capture_ids_.count(goal->request_id) != 0))
      {
        std::lock_guard<std::mutex> group_lock(group_reservations_mutex_);
        auto active = active_group_captures_.find(goal->group_id);
        if (active != active_group_captures_.end() && --active->second == 0) {
          active_group_captures_.erase(active);
        }
        return rclcpp_action::GoalResponse::REJECT;
      }
      ++active_capture_count_;
      if (!goal->request_id.empty()) {active_capture_ids_.insert(goal->request_id);}
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> handle)
  {
    std::thread([this, handle]() {execute(handle);}).detach();
  }

  void feedback(
    const std::shared_ptr<GoalHandle> & handle, const std::string & stage,
    const std::string & detail, std::uint8_t progress,
    const std::vector<std::string> & received = {},
    const std::vector<std::string> & pending = {})
  {
    auto value = std::make_shared<RecordCapture::Feedback>();
    value->stage = stage;
    value->detail = detail;
    value->progress_percent = progress;
    value->received_sensor_ids = received;
    value->pending_sensor_ids = pending;
    handle->publish_feedback(value);
  }

  void ensure_capture_subscriptions_locked(
    const vixel_interfaces::msg::Sensor & sensor)
  {
    if (sensor.topic_base.empty() ||
      capture_image_subscriptions_.count(sensor.sensor_id) != 0)
    {
      return;
    }
    const auto sensor_id = sensor.sensor_id;
    const auto capture_qos = rclcpp::QoS(rclcpp::KeepLast(256)).reliable();
    rclcpp::SubscriptionOptions options;
    options.callback_group = capture_callback_group_;
    capture_image_subscriptions_[sensor_id] =
      create_subscription<vixel_interfaces::msg::CaptureFrameChunk>(
      sensor.topic_base + "/image_capture/chunks", capture_qos,
      [frames = capture_frames_, sensor_id](
        vixel_interfaces::msg::CaptureFrameChunk::ConstSharedPtr chunk) {
        std::lock_guard<std::mutex> lock(frames->mutex);
        accept_chunk(*frames, sensor_id, *chunk);
      }, options);
    capture_info_subscriptions_[sensor_id] =
      create_subscription<sensor_msgs::msg::CameraInfo>(
      sensor.topic_base + "/camera_info", rclcpp::SensorDataQoS(),
      [frames = capture_frames_, sensor_id](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        std::lock_guard<std::mutex> lock(frames->mutex);
        append_bounded(frames->camera_info[sensor_id], std::move(info));
        frames->changed.notify_all();
      }, options);
  }

  CaptureSubscriptions subscribe(const std::vector<std::string> & sensor_ids)
  {
    CaptureSubscriptions result;
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    result.frames = capture_frames_;
    for (const auto & sensor_id : sensor_ids) {
      auto sensor = sensors_.find(sensor_id);
      if (sensor == sensors_.end() || sensor->second.topic_base.empty()) {
        throw std::runtime_error("no image topic for sensor " + sensor_id);
      }
      ensure_capture_subscriptions_locked(sensor->second);
      result.image_subscriptions[sensor_id] = capture_image_subscriptions_.at(sensor_id);
      result.info_subscriptions[sensor_id] = capture_info_subscriptions_.at(sensor_id);
    }
    return result;
  }

  void execute(const std::shared_ptr<GoalHandle> & handle)
  {
    const auto execution_started = std::chrono::steady_clock::now();
    auto acquisition_finished = execution_started;
    struct ActiveGuard
    {
      std::mutex & mutex;
      std::size_t & count;
      std::set<std::string> & capture_ids;
      std::string capture_id;
      std::mutex & group_mutex;
      std::map<std::string, std::size_t> & active_groups;
      std::string group_id;
      bool released{false};

      void release()
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (released) {return;}
        if (count != 0) {--count;}
        if (!capture_id.empty()) {capture_ids.erase(capture_id);}
        {
          std::lock_guard<std::mutex> group_lock(group_mutex);
          auto active = active_groups.find(group_id);
          if (active != active_groups.end() && --active->second == 0) {
            active_groups.erase(active);
          }
        }
        released = true;
      }

      ~ActiveGuard()
      {
        release();
      }
    } guard{
      active_captures_mutex_, active_capture_count_, active_capture_ids_,
      handle->get_goal()->request_id, group_reservations_mutex_,
      active_group_captures_, handle->get_goal()->group_id};

    auto result = std::make_shared<RecordCapture::Result>();
    CaptureRecord record;
    nlohmann::json gps_metadata = nlohmann::json::object();
    record.group_id = handle->get_goal()->group_id;
    record.operation_id = handle->get_goal()->operation_id;
    record.cycle = handle->get_goal()->cycle;
    record.metadata_json = handle->get_goal()->metadata_json;
    record.started_at = utc_now();
    std::filesystem::path staging;
    try {
      vixel_interfaces::msg::SyncGroup group;
      std::map<std::string, vixel_interfaces::msg::Sensor> sensor_snapshot;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto group_iterator = groups_.find(record.group_id);
        if (group_iterator == groups_.end()) {
          throw std::runtime_error("unknown synchronization group " + record.group_id);
        }
        group = group_iterator->second;
        sensor_snapshot = sensors_;
      }
      if (group.operating_mode != "capture") {
        throw std::runtime_error("set synchronization group to capture mode first");
      }
      if (!group.ready) {
        throw std::runtime_error(
                group.last_error.empty() ? "synchronization group is not ready" :
                group.last_error);
      }
      record.requested_sensor_ids = group.member_ids;
      record.capture_id = handle->get_goal()->request_id.empty() ?
        generated_capture_id(++capture_sequence_) : handle->get_goal()->request_id;
      if (!safe_identifier(record.capture_id)) {
        throw std::runtime_error("capture ID contains unsafe characters");
      }
      const auto space = std::filesystem::space(config_.root_directory);
      if (space.available < config_.minimum_free_bytes) {
        throw std::runtime_error("capture rejected: recording disk is below free-space threshold");
      }
      feedback(handle, "subscribing", "Waiting for full-resolution image publishers", 10);
      auto subscriptions = subscribe(group.member_ids);
      const auto match_deadline = std::chrono::steady_clock::now() + 2s;
      while (std::chrono::steady_clock::now() < match_deadline) {
        if (handle->is_canceling()) {throw std::runtime_error("capture cancelled");}
        const bool matched = std::all_of(
          subscriptions.image_subscriptions.begin(), subscriptions.image_subscriptions.end(),
          [](const auto & subscription) {
            return subscription.second->get_publisher_count() != 0;
          });
        if (matched) {break;}
        std::this_thread::sleep_for(50ms);
      }
      if (!std::all_of(
          subscriptions.image_subscriptions.begin(), subscriptions.image_subscriptions.end(),
          [](const auto & subscription) {
            return subscription.second->get_publisher_count() != 0;
          }))
      {
        throw std::runtime_error("one or more full-resolution image publishers are unavailable");
      }
      if (!capture_client_->wait_for_service(3s)) {
        throw std::runtime_error("capture group service is unavailable");
      }
      feedback(handle, "triggering", "Queuing grouped camera capture", 20);
      auto request = std::make_shared<CaptureGroup::Request>();
      request->group_id = record.group_id;
      request->request_id = record.capture_id;
      request->trigger_only = false;
      request->has_requested_time = handle->get_goal()->has_requested_time;
      request->requested_time = handle->get_goal()->requested_time;
      auto response_future = capture_client_->async_send_request(request);
      if (response_future.wait_for(config_.capture_timeout + 5s) != std::future_status::ready) {
        throw std::runtime_error("capture group service timed out");
      }
      const auto response = response_future.get();
      record.scheduled_time = response->scheduled_time;
      record.trigger_span_ns = response->trigger_span_ns;
      record.exposure_skew_ns = response->exposure_skew_ns;
      record.within_tolerance = response->within_tolerance;
      record.camera_timings = response->camera_timings;
      record.participating_sensor_ids = response->participating_sensor_ids;
      record.missing_sensor_ids = response->missing_sensor_ids;
      if (!response->success) {throw std::runtime_error(response->message);}
      const auto synchronized_count = std::count_if(
        response->camera_timings.begin(), response->camera_timings.end(),
        [](const auto & timing) {return timing.synchronized;});
      if (synchronized_count > 1 && !response->within_tolerance) {
        throw std::runtime_error(
                "provider returned out-of-tolerance synchronized frames");
      }
      if (record.participating_sensor_ids.empty()) {
        throw std::runtime_error("capture has no participating sensors");
      }
      if (config_.gps_enabled) {gps_metadata = gps_for_capture(record.scheduled_time);}

      feedback(
        handle, "waiting", "Waiting for captured image messages", 35, {},
        record.participating_sensor_ids);
      std::map<std::string, sensor_msgs::msg::CompressedImage::ConstSharedPtr> captured;
      std::map<std::string, sensor_msgs::msg::CameraInfo::ConstSharedPtr> captured_info;
      std::size_t reported_count = 0;
      const auto deadline = std::chrono::steady_clock::now() + config_.capture_timeout;
      while (std::chrono::steady_clock::now() < deadline) {
        {
          std::unique_lock<std::mutex> lock(subscriptions.frames->mutex);
          subscriptions.frames->changed.wait_for(lock, 100ms);
          for (const auto & sensor_id : record.participating_sensor_ids) {
            const auto image = subscriptions.frames->images[sensor_id].find(record.capture_id);
            if (image != subscriptions.frames->images[sensor_id].end()) {
              captured[sensor_id] = image->second;
            }
            for (const auto & info : subscriptions.frames->camera_info[sensor_id]) {
              if (same_stamp(info->header.stamp, record.scheduled_time)) {
                captured_info[sensor_id] = info;
              }
            }
          }
          if (captured.size() == record.participating_sensor_ids.size()) {break;}
          std::vector<std::string> received, pending;
          for (const auto & sensor_id : record.participating_sensor_ids) {
            (captured.count(sensor_id) ? received : pending).push_back(sensor_id);
          }
          if (received.size() != reported_count) {
            reported_count = received.size();
            const auto progress = static_cast<std::uint8_t>(
              35 + (30 * received.size()) / record.participating_sensor_ids.size());
            feedback(
              handle, "waiting", "Receiving captured image messages", progress,
              received, pending);
          }
        }
        if (handle->is_canceling()) {throw std::runtime_error("capture cancelled");}
      }
      if (captured.size() != record.participating_sensor_ids.size()) {
        for (const auto & sensor_id : record.participating_sensor_ids) {
          if (captured.count(sensor_id) == 0 &&
            std::find(
              record.missing_sensor_ids.begin(), record.missing_sensor_ids.end(), sensor_id) ==
            record.missing_sensor_ids.end())
          {
            record.missing_sensor_ids.push_back(sensor_id);
          }
        }
        throw std::runtime_error("timed out waiting for one or more captured images");
      }
      acquisition_finished = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(subscriptions.frames->mutex);
        for (const auto & sensor_id : record.participating_sensor_ids) {
          subscriptions.frames->images[sensor_id].erase(record.capture_id);
          subscriptions.frames->assemblies[sensor_id].erase(record.capture_id);
        }
      }

      const auto date = record.started_at.substr(0, 10);
      const auto parent = config_.root_directory / date.substr(0, 4) / date.substr(5, 2) /
        date.substr(8, 2);
      std::filesystem::create_directories(parent);
      staging = parent / ("." + record.capture_id + ".tmp");
      const auto final_directory = parent / record.capture_id;
      const auto failed_directory = parent / (record.capture_id + ".failed");
      if (std::filesystem::exists(staging) || std::filesystem::exists(final_directory) ||
        std::filesystem::exists(failed_directory))
      {
        throw std::runtime_error("capture ID already exists on disk");
      }
      std::filesystem::create_directory(staging);
      feedback(handle, "writing", "Writing full-resolution PNG images", 70);
      nlohmann::json sensor_manifest = nlohmann::json::object();
      for (const auto & sensor_id : record.participating_sensor_ids) {
        const auto image = captured.at(sensor_id);
        if (image->format.find("png") == std::string::npos) {
          throw std::runtime_error(
                  "capture transport for " + sensor_id + " is not lossless PNG");
        }
        const auto dimensions = png_dimensions(image->data);
        const auto filename = sensor_id + ".png";
        const auto path = staging / filename;
        // Capture transport is already a lossless PNG. Re-encoding it here is
        // both redundant and expensive for full-resolution frames.
        std::ofstream output(path, std::ios::binary);
        output.write(
          reinterpret_cast<const char *>(image->data.data()),
          static_cast<std::streamsize>(image->data.size()));
        if (!output) {throw std::runtime_error("failed to write PNG for " + sensor_id);}
        record.saved_sensor_ids.push_back(sensor_id);
        const auto sensor = sensor_snapshot.find(sensor_id);
        nlohmann::json metadata{
          {"file", filename}, {"width", dimensions.first}, {"height", dimensions.second},
          {"encoding", "bgr8"}, {"transport_format", image->format},
          {"frame_id", image->header.frame_id},
          {"stamp", stamp_json(image->header.stamp)}
        };
        if (sensor != sensor_snapshot.end()) {
          metadata["vendor"] = sensor->second.vendor;
          metadata["model"] = sensor->second.model;
          metadata["serial"] = sensor->second.serial;
          metadata["display_name"] = sensor->second.display_name;
          metadata["location_label"] = sensor->second.location_label;
          metadata["calibration_url"] = sensor->second.calibration_url;
          metadata["applied_settings_json"] = sensor->second.applied_settings_json;
        }
        if (captured_info.count(sensor_id) != 0) {
          metadata["camera_info"] = camera_info_json(*captured_info.at(sensor_id));
        }
        sensor_manifest[sensor_id] = std::move(metadata);
      }
      record.status = "complete";
      record.completed_at = utc_now();
      record.directory = final_directory.string();
      record.message = response->message + "; images saved";
      const auto writing_finished = std::chrono::steady_clock::now();
      record.timings_json = nlohmann::json({
        {"acquire_and_receive_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
            acquisition_finished - execution_started).count()},
        {"write_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
            writing_finished - acquisition_finished).count()},
        {"total_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
            writing_finished - execution_started).count()}
      }).dump();
      write_manifest(staging, record, sensor_manifest, gps_metadata);
      std::filesystem::rename(staging, final_directory);
      staging.clear();
      add_record(record);
      result->success = true;
      result->message = record.message;
      result->capture_id = record.capture_id;
      result->directory = record.directory;
      result->saved_sensor_ids = record.saved_sensor_ids;
      result->missing_sensor_ids = record.missing_sensor_ids;
      feedback(handle, "complete", "Capture set saved", 100, record.saved_sensor_ids, {});
      handle->succeed(result);
      return;
    } catch (const std::exception & error) {
      if (!record.capture_id.empty()) {
        std::lock_guard<std::mutex> lock(capture_frames_->mutex);
        for (const auto & sensor_id : record.requested_sensor_ids) {
          capture_frames_->images[sensor_id].erase(record.capture_id);
          capture_frames_->assemblies[sensor_id].erase(record.capture_id);
        }
      }
      record.status = handle->is_canceling() ? "cancelled" : "failed";
      record.completed_at = utc_now();
      record.message = error.what();
      preserve_failure(staging, record);
      add_record(record);
      result->success = false;
      result->message = record.message;
      result->capture_id = record.capture_id;
      result->directory = record.directory;
      result->saved_sensor_ids = record.saved_sensor_ids;
      result->missing_sensor_ids = record.missing_sensor_ids;
      if (handle->is_canceling()) {
        handle->canceled(result);
      } else {
        handle->abort(result);
      }
    }
  }

  void write_manifest(
    const std::filesystem::path & directory, const CaptureRecord & record,
    const nlohmann::json & sensors,
    const nlohmann::json & gps = nlohmann::json::object())
  {
    nlohmann::json timings = nlohmann::json::array();
    for (const auto & timing : record.camera_timings) {
      timings.push_back({
        {"sensor_id", timing.sensor_id},
        {"device_timestamp_ns", timing.device_timestamp_ns},
        {"ptp_offset_ns", timing.ptp_offset_ns},
        {"synchronized", timing.synchronized},
      });
    }
    nlohmann::json metadata = nlohmann::json::object();
    nlohmann::json capture_timings = nlohmann::json::object();
    if (!record.metadata_json.empty()) {metadata = nlohmann::json::parse(record.metadata_json);}
    if (!record.timings_json.empty()) {
      capture_timings = nlohmann::json::parse(record.timings_json);
    }
    nlohmann::json manifest{
      {"schema_version", 2}, {"capture_id", record.capture_id},
      {"group_id", record.group_id}, {"status", record.status},
      {"operation_id", record.operation_id}, {"cycle", record.cycle},
      {"metadata", metadata}, {"capture_timings", capture_timings},
      {"message", record.message}, {"started_at", record.started_at},
      {"completed_at", record.completed_at},
      {"scheduled_time", stamp_json(record.scheduled_time)},
      {"trigger_span_ns", record.trigger_span_ns},
      {"exposure_skew_ns", record.exposure_skew_ns},
      {"within_tolerance", record.within_tolerance},
      {"camera_timings", timings},
      {"requested_sensor_ids", record.requested_sensor_ids},
      {"participating_sensor_ids", record.participating_sensor_ids},
      {"saved_sensor_ids", record.saved_sensor_ids},
      {"missing_sensor_ids", record.missing_sensor_ids},
      {"synchronization", "ptp_scheduled_action_with_software_fallback"},
      {"sensors", sensors}
    };
    if (!gps.empty()) {manifest["gps"] = gps;}
    std::ofstream output(directory / "manifest.json");
    output << std::setw(2) << manifest << '\n';
    if (!output) {throw std::runtime_error("failed to write capture manifest");}
  }

  void preserve_failure(std::filesystem::path & staging, CaptureRecord & record)
  {
    if (record.capture_id.empty()) {return;}
    try {
      if (staging.empty()) {
        const auto date = record.started_at.substr(0, 10);
        const auto parent = config_.root_directory / date.substr(0, 4) / date.substr(5, 2) /
          date.substr(8, 2);
        std::filesystem::create_directories(parent);
        const auto failed = parent / (record.capture_id + ".failed");
        const auto complete = parent / record.capture_id;
        if (std::filesystem::exists(failed) || std::filesystem::exists(complete)) {
          record.directory.clear();
          return;
        }
        staging = parent / ("." + record.capture_id + ".tmp");
        if (!std::filesystem::exists(staging)) {std::filesystem::create_directory(staging);}
      }
      write_manifest(staging, record, nlohmann::json::object());
      const auto failed = staging.parent_path() / (record.capture_id + ".failed");
      if (std::filesystem::exists(failed)) {
        std::filesystem::remove_all(staging);
        record.directory.clear();
        staging.clear();
        return;
      }
      std::filesystem::rename(staging, failed);
      record.directory = failed.string();
      staging.clear();
    } catch (const std::exception & preserve_error) {
      RCLCPP_ERROR(get_logger(), "Unable to preserve failed capture: %s", preserve_error.what());
    }
  }

  void add_record(CaptureRecord record)
  {
    record.stamp = now();
    {
      std::lock_guard<std::mutex> lock(records_mutex_);
      records_.insert(records_.begin(), std::move(record));
      if (records_.size() > config_.recent_limit) {records_.resize(config_.recent_limit);}
      ++records_generation_;
    }
    publish_records();
  }

  void publish_records()
  {
    if (!records_publisher_) {return;}
    vixel_interfaces::msg::CaptureRecordArray message;
    message.header.stamp = now();
    {
      std::lock_guard<std::mutex> lock(records_mutex_);
      message.generation = records_generation_;
      message.records = records_;
    }
    records_publisher_->publish(message);
  }

  static CaptureRecord record_from_json(
    const nlohmann::json & value, const std::filesystem::path & directory)
  {
    CaptureRecord record;
    record.capture_id = value.value("capture_id", directory.filename().string());
    record.group_id = value.value("group_id", "");
    record.operation_id = value.value("operation_id", "");
    record.cycle = value.value("cycle", 0U);
    record.metadata_json = value.value("metadata", nlohmann::json::object()).dump();
    record.timings_json = value.value("capture_timings", nlohmann::json::object()).dump();
    record.status = value.value("status", "unknown");
    record.directory = directory.string();
    record.message = value.value("message", "");
    record.started_at = value.value("started_at", "");
    record.completed_at = value.value("completed_at", "");
    record.requested_sensor_ids = value.value(
      "requested_sensor_ids", std::vector<std::string>{});
    record.participating_sensor_ids = value.value(
      "participating_sensor_ids", std::vector<std::string>{});
    record.saved_sensor_ids = value.value("saved_sensor_ids", std::vector<std::string>{});
    record.missing_sensor_ids = value.value("missing_sensor_ids", std::vector<std::string>{});
    const auto scheduled = value.value("scheduled_time", nlohmann::json::object());
    record.scheduled_time.sec = scheduled.value("sec", 0);
    record.scheduled_time.nanosec = scheduled.value("nanosec", 0U);
    record.trigger_span_ns = value.value("trigger_span_ns", 0ULL);
    record.exposure_skew_ns = value.value("exposure_skew_ns", 0ULL);
    record.within_tolerance = value.value("within_tolerance", false);
    for (const auto & item : value.value("camera_timings", nlohmann::json::array())) {
      vixel_interfaces::msg::CameraTiming timing;
      timing.sensor_id = item.value("sensor_id", "");
      timing.device_timestamp_ns = item.value("device_timestamp_ns", 0ULL);
      timing.ptp_offset_ns = item.value("ptp_offset_ns", 0LL);
      timing.synchronized = item.value("synchronized", false);
      record.camera_timings.push_back(timing);
    }
    return record;
  }

  void scan_records()
  {
    std::vector<CaptureRecord> loaded;
    for (const auto & item : std::filesystem::recursive_directory_iterator(
        config_.root_directory, std::filesystem::directory_options::skip_permission_denied))
    {
      if (!item.is_regular_file() || item.path().filename() != "manifest.json") {continue;}
      try {
        std::ifstream input(item.path());
        nlohmann::json value;
        input >> value;
        loaded.push_back(record_from_json(value, item.path().parent_path()));
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(), "Ignoring unreadable capture manifest %s: %s",
          item.path().c_str(), error.what());
      }
    }
    std::sort(loaded.begin(), loaded.end(), [](const auto & left, const auto & right) {
      return left.completed_at > right.completed_at;
    });
    if (loaded.size() > config_.recent_limit) {loaded.resize(config_.recent_limit);}
    records_ = std::move(loaded);
    records_generation_ = 1;
  }

  RecorderConfig config_;
  std::atomic<std::uint64_t> capture_sequence_{0};
  std::atomic<std::uint64_t> operation_sequence_{0};
  std::mutex active_captures_mutex_;
  std::size_t active_capture_count_{0};
  std::set<std::string> active_capture_ids_;
  std::mutex group_reservations_mutex_;
  std::map<std::string, std::string> group_reservations_;
  std::map<std::string, std::size_t> active_group_captures_;
  std::mutex state_mutex_;
  std::condition_variable state_changed_;
  std::map<std::string, vixel_interfaces::msg::Sensor> sensors_;
  std::map<std::string, vixel_interfaces::msg::SyncGroup> groups_;
  std::shared_ptr<FrameBucket> capture_frames_{std::make_shared<FrameBucket>()};
  std::map<
    std::string,
    rclcpp::Subscription<vixel_interfaces::msg::CaptureFrameChunk>::SharedPtr>
  capture_image_subscriptions_;
  std::map<
    std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>
  capture_info_subscriptions_;
  rclcpp::CallbackGroup::SharedPtr capture_callback_group_;
  std::mutex records_mutex_;
  std::vector<CaptureRecord> records_;
  std::uint64_t records_generation_{0};
  rclcpp::Subscription<vixel_interfaces::msg::SensorArray>::SharedPtr sensors_subscription_;
  rclcpp::Subscription<vixel_interfaces::msg::SyncGroupArray>::SharedPtr groups_subscription_;
  std::mutex gps_mutex_;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr latest_gps_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Client<CaptureGroup>::SharedPtr capture_client_;
  rclcpp::Client<PrepareCaptureGroups>::SharedPtr prepare_capture_groups_client_;
  rclcpp::Publisher<vixel_interfaces::msg::CaptureRecordArray>::SharedPtr records_publisher_;
  rclcpp_action::Server<RecordCapture>::SharedPtr action_server_;
  rclcpp_action::Client<RecordCapture>::SharedPtr operation_capture_client_;
  std::mutex operations_mutex_;
  std::map<std::string, std::shared_ptr<OperationState>> operations_;
  std::deque<std::string> terminal_operation_ids_;
  std::uint64_t operations_generation_{0};
  rclcpp::Publisher<vixel_interfaces::msg::CaptureOperationArray>::SharedPtr
  operations_publisher_;
  rclcpp::Service<vixel_interfaces::srv::SubmitCaptureBatch>::SharedPtr submit_batch_service_;
  rclcpp::Service<vixel_interfaces::srv::StartCaptureSequence>::SharedPtr start_sequence_service_;
  rclcpp::Service<vixel_interfaces::srv::GetCaptureOperation>::SharedPtr get_operation_service_;
  rclcpp::Service<vixel_interfaces::srv::CancelCaptureOperation>::SharedPtr
  cancel_operation_service_;
};

}  // namespace vixel_recorder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vixel_recorder::CaptureRecorder>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
