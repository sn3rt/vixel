#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vixel_interfaces/action/record_capture.hpp>
#include <vixel_interfaces/msg/capture_frame_chunk.hpp>
#include <vixel_interfaces/msg/capture_record.hpp>
#include <vixel_interfaces/msg/capture_record_array.hpp>
#include <vixel_interfaces/msg/sensor_array.hpp>
#include <vixel_interfaces/msg/sync_group_array.hpp>
#include <vixel_interfaces/srv/capture_group.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
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
using RecordCapture = vixel_interfaces::action::RecordCapture;
using GoalHandle = rclcpp_action::ServerGoalHandle<RecordCapture>;

struct RecorderConfig
{
  std::filesystem::path root_directory{"/var/lib/vixel/captures"};
  std::uintmax_t minimum_free_bytes{5ULL * 1024ULL * 1024ULL * 1024ULL};
  std::chrono::milliseconds capture_timeout{10000};
  int png_compression{3};
  std::size_t recent_limit{100};
};

template<typename T>
T read_or(const YAML::Node & node, const char * key, T fallback)
{
  return node && node[key] ? node[key].as<T>() : fallback;
}

RecorderConfig load_config(const std::string & path)
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
  result.png_compression = read_or(recording, "png_compression", result.png_compression);
  result.recent_limit = read_or(recording, "recent_limit", result.recent_limit);
  if (result.root_directory.empty()) {throw std::runtime_error("recording root is empty");}
  if (result.capture_timeout < 1s || result.capture_timeout > 60s) {
    throw std::runtime_error("recording capture timeout must be between 1 and 60 seconds");
  }
  if (result.png_compression < 0 || result.png_compression > 9) {
    throw std::runtime_error("recording PNG compression must be between 0 and 9");
  }
  return result;
}

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
  std::map<std::string, std::vector<sensor_msgs::msg::CompressedImage::ConstSharedPtr>> images;
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
  std::vector<sensor_msgs::msg::CompressedImage::ConstSharedPtr> & values,
  sensor_msgs::msg::CompressedImage::ConstSharedPtr value)
{
  values.push_back(std::move(value));
  if (values.size() > 4) {values.erase(values.begin());}
}

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
  auto & assembly = frames.assemblies[sensor_id][chunk.capture_id];
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
  append_bounded(frames.images[sensor_id], std::move(image));
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
    config_ = load_config(machine_file);
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
      });
    capture_client_ = create_client<CaptureGroup>("/vixel/capture_group");
    capture_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    records_publisher_ = create_publisher<vixel_interfaces::msg::CaptureRecordArray>(
      "/vixel/capture_records", state_qos);
    action_server_ = rclcpp_action::create_server<RecordCapture>(
      this, "/vixel/record_capture",
      std::bind(&CaptureRecorder::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CaptureRecorder::handle_cancel, this, std::placeholders::_1),
      std::bind(&CaptureRecorder::handle_accepted, this, std::placeholders::_1));
    publish_records();
    RCLCPP_INFO(
      get_logger(), "Capture recorder ready at %s (minimum free %.2f GiB)",
      config_.root_directory.c_str(),
      static_cast<double>(config_.minimum_free_bytes) / (1024.0 * 1024.0 * 1024.0));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const RecordCapture::Goal> goal)
  {
    if (goal->group_id.empty()) {return rclcpp_action::GoalResponse::REJECT;}
    if (!goal->request_id.empty() && !safe_identifier(goal->request_id)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::vector<std::string> member_ids;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto group = groups_.find(goal->group_id);
      if (group == groups_.end() || group->second.member_ids.empty()) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      member_ids = group->second.member_ids;
    }
    {
      std::lock_guard<std::mutex> lock(active_captures_mutex_);
      if (active_capture_groups_.count(goal->group_id) != 0 ||
        std::any_of(member_ids.begin(), member_ids.end(), [this](const auto & sensor_id) {
          return active_capture_sensors_.count(sensor_id) != 0;
        }))
      {
        return rclcpp_action::GoalResponse::REJECT;
      }
      active_capture_groups_[goal->group_id] = member_ids;
      active_capture_sensors_.insert(member_ids.begin(), member_ids.end());
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
    const auto capture_qos = rclcpp::QoS(rclcpp::KeepLast(128)).reliable();
    rclcpp::SubscriptionOptions options;
    options.callback_group = capture_callback_group_;
    capture_image_subscriptions_[sensor_id] =
      create_subscription<vixel_interfaces::msg::CaptureFrameChunk>(
      sensor.topic_base + "/image_capture/chunks", capture_qos,
      [](vixel_interfaces::msg::CaptureFrameChunk::ConstSharedPtr) {}, options);
    capture_info_subscriptions_[sensor_id] =
      create_subscription<sensor_msgs::msg::CameraInfo>(
      sensor.topic_base + "/camera_info", rclcpp::SensorDataQoS(),
      [](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {}, options);
  }

  CaptureSubscriptions subscribe(const std::vector<std::string> & sensor_ids)
  {
    CaptureSubscriptions result;
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    result.frames = capture_frames_;
    {
      std::lock_guard<std::mutex> frames_lock(capture_frames_->mutex);
      for (const auto & sensor_id : sensor_ids) {
        capture_frames_->images.erase(sensor_id);
        capture_frames_->assemblies.erase(sensor_id);
        capture_frames_->camera_info.erase(sensor_id);
      }
    }
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
    struct ActiveGuard
    {
      std::mutex & mutex;
      std::map<std::string, std::vector<std::string>> & groups;
      std::set<std::string> & sensors;
      std::string group_id;

      ~ActiveGuard()
      {
        std::lock_guard<std::mutex> lock(mutex);
        const auto group = groups.find(group_id);
        if (group == groups.end()) {return;}
        for (const auto & sensor_id : group->second) {sensors.erase(sensor_id);}
        groups.erase(group);
      }
    } guard{
      active_captures_mutex_, active_capture_groups_, active_capture_sensors_,
      handle->get_goal()->group_id};

    auto result = std::make_shared<RecordCapture::Result>();
    CaptureRecord record;
    record.group_id = handle->get_goal()->group_id;
    record.started_at = utc_now();
    std::filesystem::path staging;
    try {
      std::vector<std::string> reserved_member_ids;
      {
        std::lock_guard<std::mutex> lock(active_captures_mutex_);
        const auto reservation = active_capture_groups_.find(record.group_id);
        if (reservation == active_capture_groups_.end()) {
          throw std::runtime_error("capture reservation is missing");
        }
        reserved_member_ids = reservation->second;
      }
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
      if (group.member_ids != reserved_member_ids) {
        throw std::runtime_error("synchronization group changed while capture was starting");
      }
      if (group.operating_mode != "capture") {
        throw std::runtime_error("set synchronization group to capture mode first");
      }
      if (!group.ready) {
        throw std::runtime_error("synchronization group is not ready");
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
      if (record.participating_sensor_ids.empty()) {
        throw std::runtime_error("capture has no participating sensors");
      }

      feedback(
        handle, "waiting", "Waiting for captured image messages", 35, {},
        record.participating_sensor_ids);
      std::map<std::string, sensor_msgs::msg::CompressedImage::ConstSharedPtr> captured;
      std::map<std::string, sensor_msgs::msg::CameraInfo::ConstSharedPtr> captured_info;
      std::size_t reported_count = 0;
      const auto deadline = std::chrono::steady_clock::now() + config_.capture_timeout;
      rclcpp::WaitSet wait_set;
      struct WaitSetSubscriptions
      {
        rclcpp::WaitSet & wait_set;
        std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions;

        void add(const std::shared_ptr<rclcpp::SubscriptionBase> & subscription)
        {
          wait_set.add_subscription(subscription);
          subscriptions.push_back(subscription);
        }

        ~WaitSetSubscriptions()
        {
          for (
            auto iterator = subscriptions.rbegin();
            iterator != subscriptions.rend(); ++iterator)
          {
            try {
              wait_set.remove_subscription(*iterator);
            } catch (...) {
              // Cleanup must not mask the capture result during stack unwinding.
            }
          }
        }
      } wait_set_subscriptions{wait_set, {}};
      for (const auto & subscription : subscriptions.image_subscriptions) {
        wait_set_subscriptions.add(subscription.second);
      }
      for (const auto & subscription : subscriptions.info_subscriptions) {
        wait_set_subscriptions.add(subscription.second);
      }
      std::size_t chunks_taken = 0;
      while (std::chrono::steady_clock::now() < deadline) {
        const auto wait_result = wait_set.wait(100ms);
        (void)wait_result;
        for (const auto & sensor_id : record.participating_sensor_ids) {
          vixel_interfaces::msg::CaptureFrameChunk chunk;
          rclcpp::MessageInfo message_info;
          while (subscriptions.image_subscriptions.at(sensor_id)->take(chunk, message_info)) {
            if (++chunks_taken == 1) {
              RCLCPP_INFO(
                get_logger(), "Receiving capture %s chunks",
                record.capture_id.c_str());
            }
            std::lock_guard<std::mutex> lock(subscriptions.frames->mutex);
            accept_chunk(*subscriptions.frames, sensor_id, chunk);
          }
          sensor_msgs::msg::CameraInfo info;
          while (subscriptions.info_subscriptions.at(sensor_id)->take(info, message_info)) {
            std::lock_guard<std::mutex> lock(subscriptions.frames->mutex);
            append_bounded(
              subscriptions.frames->camera_info[sensor_id],
              std::make_shared<sensor_msgs::msg::CameraInfo>(info));
          }
        }
        {
          std::lock_guard<std::mutex> lock(subscriptions.frames->mutex);
          for (const auto & sensor_id : record.participating_sensor_ids) {
            for (const auto & image : subscriptions.frames->images[sensor_id]) {
              if (same_stamp(image->header.stamp, record.scheduled_time)) {
                captured[sensor_id] = image;
              }
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
      feedback(handle, "writing", "Encoding full-resolution PNG images", 70);
      nlohmann::json sensor_manifest = nlohmann::json::object();
      for (const auto & sensor_id : record.participating_sensor_ids) {
        const auto image = captured.at(sensor_id);
        if (image->format.find("png") == std::string::npos) {
          throw std::runtime_error(
                  "capture transport for " + sensor_id + " is not lossless PNG");
        }
        const cv::Mat encoded(
          1, static_cast<int>(image->data.size()), CV_8UC1,
          const_cast<std::uint8_t *>(image->data.data()));
        const auto decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (decoded.empty()) {
          throw std::runtime_error("failed to decode full-resolution PNG for " + sensor_id);
        }
        const auto filename = sensor_id + ".png";
        const auto path = staging / filename;
        if (!cv::imwrite(
            path.string(), decoded,
            {cv::IMWRITE_PNG_COMPRESSION, config_.png_compression}))
        {
          throw std::runtime_error("failed to encode PNG for " + sensor_id);
        }
        record.saved_sensor_ids.push_back(sensor_id);
        const auto sensor = sensor_snapshot.find(sensor_id);
        nlohmann::json metadata{
          {"file", filename}, {"width", decoded.cols}, {"height", decoded.rows},
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
      write_manifest(staging, record, sensor_manifest);
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
    const nlohmann::json & sensors)
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
    nlohmann::json manifest{
      {"schema_version", 1}, {"capture_id", record.capture_id},
      {"group_id", record.group_id}, {"status", record.status},
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
  std::mutex active_captures_mutex_;
  std::map<std::string, std::vector<std::string>> active_capture_groups_;
  std::set<std::string> active_capture_sensors_;
  std::mutex state_mutex_;
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
  rclcpp::Client<CaptureGroup>::SharedPtr capture_client_;
  rclcpp::Publisher<vixel_interfaces::msg::CaptureRecordArray>::SharedPtr records_publisher_;
  rclcpp_action::Server<RecordCapture>::SharedPtr action_server_;
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
