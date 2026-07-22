#include <vixel_lucid/LucidConfig.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vixel_interfaces/msg/provider_assignment_array.hpp>
#include <vixel_interfaces/msg/sensor_array.hpp>
#include <vixel_interfaces/msg/sensor_observation_array.hpp>
#include <vixel_interfaces/msg/sync_group_array.hpp>
#include <vixel_interfaces/srv/provider_capture.hpp>
#include <vixel_interfaces/srv/provision_sensor.hpp>

#include <ArenaApi.h>
#include <GenICam.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/file.h>
#include <unistd.h>

namespace vixel_lucid
{
namespace
{
using namespace std::chrono_literals;
using Assignment = vixel_interfaces::msg::ProviderAssignment;
using Sensor = vixel_interfaces::msg::Sensor;
using SyncGroup = vixel_interfaces::msg::SyncGroup;

std::string current_exception_message()
{
  try {
    throw;
  } catch (const GenICam::GenericException & error) {
    return error.what();
  } catch (const std::exception & error) {
    return error.what();
  } catch (...) {
    return "unknown exception";
  }
}

builtin_interfaces::msg::Time ptp_to_ros_time(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return result;
}

std::uint32_t action_group_key(const std::string & group_id, std::uint32_t seed)
{
  std::uint32_t hash = 2166136261U ^ seed;
  for (const auto character : group_id) {
    hash ^= static_cast<std::uint8_t>(character);
    hash *= 16777619U;
  }
  return hash == 0 ? 1U : hash;
}

std::string ipv4_string(std::uint32_t address)
{
  return std::to_string((address >> 24U) & 0xFFU) + "." +
         std::to_string((address >> 16U) & 0xFFU) + "." +
         std::to_string((address >> 8U) & 0xFFU) + "." +
         std::to_string(address & 0xFFU);
}

std::uint64_t ipv4_integer(const std::string & text)
{
  std::uint64_t result = 0;
  std::size_t start = 0;
  for (int index = 0; index < 4; ++index) {
    const auto end = text.find('.', start);
    if ((end == std::string::npos) != (index == 3)) {
      throw std::invalid_argument("invalid IPv4 address " + text);
    }
    const auto component = std::stoul(text.substr(start, end - start));
    if (component > 255) {
      throw std::invalid_argument("invalid IPv4 address " + text);
    }
    result = (result << 8U) | component;
    start = end == std::string::npos ? text.size() : end + 1;
  }
  return result;
}

std::uint32_t byte_swap_ipv4(std::uint32_t address)
{
  return ((address & 0x000000FFU) << 24U) |
         ((address & 0x0000FF00U) << 8U) |
         ((address & 0x00FF0000U) >> 8U) |
         ((address & 0xFF000000U) >> 24U);
}

std::uint64_t mac_integer(const std::string & text)
{
  std::string compact;
  for (const auto character : text) {
    if (std::isxdigit(static_cast<unsigned char>(character))) {
      compact.push_back(character);
    }
  }
  if (compact.empty()) {
    throw std::invalid_argument("empty MAC address");
  }
  return std::stoull(compact, nullptr, 16);
}

class CameraEndpoint
{
public:
  CameraEndpoint(rclcpp::Node * node, const Assignment & assignment, const LucidConfig & config)
  : sensor_id_(assignment.sensor_id), frame_id_(assignment.frame_id), config_(config)
  {
    const std::string base = "/vixel/sensors/" + sensor_id_;
    image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>(
      base + "/image_raw", rclcpp::SensorDataQoS());
    compressed_publisher_ = node->create_publisher<sensor_msgs::msg::CompressedImage>(
      base + "/image_raw/compressed", rclcpp::SensorDataQoS());
    info_publisher_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
      base + "/camera_info", rclcpp::SensorDataQoS());
    info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      node, sensor_id_, assignment.calibration_url, base);
  }

  void publish(
    const cv::Mat & image, const builtin_interfaces::msg::Time & stamp,
    const bool publish_raw)
  {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;
    if (publish_raw && (image_publisher_->get_subscription_count() != 0 ||
      info_publisher_->get_subscription_count() != 0))
    {
      auto image_message = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
      image_publisher_->publish(*image_message);
      auto camera_info = info_manager_->getCameraInfo();
      camera_info.header = header;
      if (camera_info.width == 0 || camera_info.height == 0) {
        camera_info.width = image_message->width;
        camera_info.height = image_message->height;
      }
      info_publisher_->publish(camera_info);
    }

    cv::Mat preview = image;
    if (image.cols > config_.preview_width) {
      const double scale = static_cast<double>(config_.preview_width) / image.cols;
      cv::resize(image, preview, cv::Size(), scale, scale, cv::INTER_AREA);
    }
    sensor_msgs::msg::CompressedImage compressed;
    compressed.header = header;
    compressed.format = "jpeg";
    const std::vector<int> options{cv::IMWRITE_JPEG_QUALITY, config_.jpeg_quality};
    if (!cv::imencode(".jpg", preview, compressed.data, options)) {
      throw std::runtime_error("OpenCV failed to encode preview JPEG");
    }
    compressed_publisher_->publish(compressed);
  }

private:
  std::string sensor_id_;
  std::string frame_id_;
  LucidConfig config_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
};

struct CaptureBatch
{
  CaptureBatch(std::size_t count, std::function<void()> callback)
  : remaining(count), done(std::move(callback)) {}

  void complete_one()
  {
    if (remaining.fetch_sub(1) == 1 && done) {
      done();
    }
  }
  std::atomic<std::size_t> remaining;
  std::function<void()> done;
};

struct CaptureCommand
{
  std::uint64_t sequence{0};
  builtin_interfaces::msg::Time stamp;
  std::chrono::steady_clock::time_point transfer_at;
  std::shared_ptr<CaptureBatch> batch;
  bool software_trigger{false};
};

class CameraSession
{
public:
  CameraSession(
    Arena::IDevice * device, Assignment assignment, NetworkConfig network,
    const LucidConfig & config, rclcpp::Node * node, std::string model,
    std::string mac, std::string address, rclcpp::Logger logger)
  : device_(device), assignment_(std::move(assignment)), network_(std::move(network)),
    config_(config), endpoint_(std::make_unique<CameraEndpoint>(node, assignment_, config)),
    node_(node), model_(std::move(model)), mac_(std::move(mac)), address_(std::move(address)),
    mode_(assignment_.operating_mode), logger_(logger)
  {
    next_preview_ = std::chrono::steady_clock::now();
  }

  ~CameraSession() {shutdown();}
  CameraSession(const CameraSession &) = delete;
  CameraSession & operator=(const CameraSession &) = delete;

  void initialize()
  {
    const auto initialization_started = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> device_lock(device_mutex_);
    auto * stream_map = device_->GetTLStreamNodeMap();
    auto * node_map = device_->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(stream_map, "StreamBufferHandlingMode", "NewestOnly");
    Arena::SetNodeValue<bool>(stream_map, "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(stream_map, "StreamPacketResendEnable", true);
    if (config_.imaging.use_max_width) {
      GenApi::CIntegerPtr width = node_map->GetNode("Width");
      width->SetValue(width->GetMax());
    }
    if (config_.imaging.use_max_height) {
      GenApi::CIntegerPtr height = node_map->GetNode("Height");
      height->SetValue(height->GetMax());
    }
    Arena::SetNodeValue<GenICam::gcstring>(
      node_map, "PixelFormat", config_.imaging.pixel_format.c_str());
    Arena::SetNodeValue<GenICam::gcstring>(
      node_map, "ExposureAuto", config_.imaging.exposure_auto.c_str());
    if (config_.imaging.exposure_auto == "Off") {
      GenApi::CFloatPtr exposure = node_map->GetNode("ExposureTime");
      exposure->SetValue(std::clamp(
        config_.imaging.exposure_time_us, exposure->GetMin(), exposure->GetMax()));
    }
    Arena::SetNodeValue<GenICam::gcstring>(
      node_map, "GainAuto", config_.imaging.gain_auto.c_str());
    if (config_.imaging.gain_auto == "Off") {
      GenApi::CFloatPtr gain = node_map->GetNode("Gain");
      gain->SetValue(std::clamp(config_.imaging.gain_db, gain->GetMin(), gain->GetMax()));
    }
    GenApi::CIntegerPtr packet_size = node_map->GetNode("DeviceStreamChannelPacketSize");
    packet_size->SetValue(std::min<std::int64_t>(packet_size->GetMax(), network_.packet_size));
    Arena::SetNodeValue<std::int64_t>(node_map, "GevSCPD", network_.packet_delay);
    Arena::SetNodeValue<GenICam::gcstring>(node_map, "TriggerSelector", "FrameStart");
    Arena::SetNodeValue<GenICam::gcstring>(node_map, "TriggerMode", "On");
    Arena::SetNodeValue<GenICam::gcstring>(
      node_map, "TriggerSource", mode_ == "preview" ? "Software" : "Action0");
    if (mode_ == "capture") {
      Arena::SetNodeValue<GenICam::gcstring>(node_map, "ActionUnconditionalMode", "On");
      Arena::SetNodeValue<std::int64_t>(node_map, "ActionSelector", 0);
      Arena::SetNodeValue<std::int64_t>(
        node_map, "ActionDeviceKey", config_.action_device_key);
      Arena::SetNodeValue<std::int64_t>(
        node_map, "ActionGroupKey",
        action_group_key(assignment_.sync_group, config_.action_group_key));
      Arena::SetNodeValue<std::int64_t>(
        node_map, "ActionGroupMask", config_.action_group_mask);
    }
    Arena::SetNodeValue<GenICam::gcstring>(node_map, "TransferControlMode", "UserControlled");
    Arena::SetNodeValue<GenICam::gcstring>(node_map, "TransferOperationMode", "Continuous");
    Arena::SetNodeValue<bool>(node_map, "PtpEnable", mode_ == "capture");
    // LUCID's scheduled-action sequence stops transfer before starting the
    // host stream. Doing this afterwards can wait on a transfer timeout.
    Arena::ExecuteNode(node_map, "TransferStop");
    const auto stream_started = std::chrono::steady_clock::now();
    device_->StartStream();
    streaming_.store(true);
    const auto stream_ready = std::chrono::steady_clock::now();
    if (mode_ == "preview") {
      ready_.store(true);
      set_ptp_status("preview (unsynchronized)");
    } else {
      set_ptp_status("waiting for group configuration");
    }
    worker_ = std::thread(&CameraSession::acquisition_loop, this);
    const auto complete = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      logger_, "Initialized %s: configuration %.3fs, StartStream %.3fs, total %.3fs",
      assignment_.sensor_id.c_str(),
      std::chrono::duration<double>(stream_started - initialization_started).count(),
      std::chrono::duration<double>(stream_ready - stream_started).count(),
      std::chrono::duration<double>(complete - initialization_started).count());
  }

  void shutdown()
  {
    bool expected = false;
    if (!stopping_.compare_exchange_strong(expected, true)) {
      return;
    }
    {
      std::lock_guard<std::mutex> capture_lock(capture_mutex_);
      if (pending_) {
        if (pending_->batch) {
          pending_->batch->complete_one();
        }
        pending_.reset();
      }
    }
    capture_cv_.notify_all();
    {
      std::lock_guard<std::mutex> device_lock(device_mutex_);
      if (streaming_.exchange(false)) {
        try {device_->StopStream();} catch (...) {}
      }
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    ready_.store(false);
  }

  void configure_ptp(bool master)
  {
    if (mode_ != "capture") {
      return;
    }
    std::lock_guard<std::mutex> device_lock(device_mutex_);
    if (configured_master_.has_value() && configured_master_.value() == master) {
      return;
    }
    ready_.store(false);
    Arena::SetNodeValue<bool>(device_->GetNodeMap(), "PtpSlaveOnly", !master);
    is_master_.store(master);
    configured_master_ = master;
    set_ptp_status(master ? "electing master" : "waiting for master");
  }

  void refresh_ptp()
  {
    if (mode_ != "capture") {
      return;
    }
    std::lock_guard<std::mutex> device_lock(device_mutex_);
    const std::string status = Arena::GetNodeValue<GenICam::gcstring>(
      device_->GetNodeMap(), "PtpStatus").c_str();
    set_ptp_status(status);
    ready_.store(streaming_.load() && (is_master_.load() ? status == "Master" : status == "Slave"));
  }

  std::int64_t latch_ptp_time()
  {
    std::lock_guard<std::mutex> device_lock(device_mutex_);
    Arena::ExecuteNode(device_->GetNodeMap(), "PtpDataSetLatch");
    return Arena::GetNodeValue<std::int64_t>(
      device_->GetNodeMap(), "PtpDataSetLatchValue");
  }

  bool preview_due(std::chrono::steady_clock::time_point now)
  {
    if (mode_ != "preview" || now < next_preview_) {
      return false;
    }
    const double rate = std::clamp(assignment_.preview_rate_hz, 0.1, 10.0);
    next_preview_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rate));
    return true;
  }

  bool queue(CaptureCommand command)
  {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (!streaming_.load() || capture_busy_ || pending_) {
      return false;
    }
    pending_ = std::move(command);
    capture_cv_.notify_one();
    return true;
  }

  bool can_capture() const
  {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    return ready_.load() && !capture_busy_ && !pending_;
  }

  Sensor status() const
  {
    Sensor result;
    result.stamp = node_->now();
    result.sensor_id = assignment_.sensor_id;
    result.provider = "lucid";
    result.kind = "camera";
    result.vendor = "LUCID";
    result.model = model_;
    result.serial = assignment_.serial;
    result.mac_address = mac_;
    result.enrolled = true;
    result.enabled = assignment_.enabled;
    result.online = streaming_.load();
    result.lifecycle_state = last_error().empty() ? (ready_.load() ? "ready" : "configuring") : "error";
    result.topic_base = "/vixel/sensors/" + assignment_.sensor_id;
    result.network_id = assignment_.network_id;
    result.assigned_address = assignment_.assigned_address;
    result.current_address = address_;
    result.sync_group = assignment_.sync_group;
    result.operating_mode = mode_;
    result.capabilities = {"image", "jpeg_preview", "ptp_action_capture"};
    result.last_error = last_error();
    return result;
  }

  Arena::IDevice * device() const {return device_;}
  const Assignment & assignment() const {return assignment_;}
  const std::string & sensor_id() const {return assignment_.sensor_id;}
  const std::string & serial() const {return assignment_.serial;}
  const std::string & network_id() const {return assignment_.network_id;}
  const std::string & mode() const {return mode_;}
  std::chrono::milliseconds transfer_slot() const
  {
    return std::chrono::milliseconds(network_.transfer_slot_ms);
  }
  bool ready() const {return ready_.load();}
  bool is_master() const {return is_master_.load();}
  std::uint64_t frames() const {return frames_.load();}
  std::uint64_t errors() const {return errors_.load();}

private:
  void set_ptp_status(const std::string & value)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ptp_status_ = value;
  }

  void set_last_error(const std::string & value)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = value;
  }

  std::string last_error() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return last_error_;
  }

  void acquisition_loop()
  {
    while (!stopping_.load()) {
      std::optional<CaptureCommand> command;
      {
        std::unique_lock<std::mutex> lock(capture_mutex_);
        capture_cv_.wait(lock, [this]() {return stopping_.load() || pending_.has_value();});
        if (stopping_.load()) {
          break;
        }
        command = std::move(pending_);
        pending_.reset();
        capture_busy_ = true;
      }
      if (command) {
        acquire(*command);
        if (command->batch) {
          command->batch->complete_one();
        }
      }
      std::lock_guard<std::mutex> lock(capture_mutex_);
      capture_busy_ = false;
    }
  }

  void acquire(const CaptureCommand & command)
  {
    std::this_thread::sleep_until(command.transfer_at);
    if (stopping_.load()) {
      return;
    }
    Arena::IImage * arena_image = nullptr;
    bool transfer_started = false;
    try {
      std::lock_guard<std::mutex> device_lock(device_mutex_);
      auto stamp = command.stamp;
      if (command.software_trigger) {
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        while (!Arena::GetNodeValue<bool>(device_->GetNodeMap(), "TriggerArmed")) {
          if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error("timed out waiting for software trigger to arm");
          }
          std::this_thread::sleep_for(1ms);
        }
        stamp = node_->now();
        Arena::ExecuteNode(device_->GetNodeMap(), "TriggerSoftware");
      }
      Arena::ExecuteNode(device_->GetNodeMap(), "TransferStart");
      transfer_started = true;
      arena_image = device_->GetImage(config_.image_timeout_ms);
      if (!arena_image) {
        throw std::runtime_error("Arena returned a null image");
      }
      if (arena_image->IsIncomplete()) {
        const auto incomplete_count = incomplete_frames_.fetch_add(1) + 1;
        throw std::runtime_error(
                "Arena returned incomplete image frame " +
                std::to_string(arena_image->GetFrameId()) + ": received " +
                std::to_string(arena_image->GetSizeFilled()) + " of " +
                std::to_string(arena_image->GetPayloadSize()) + " payload bytes (" +
                std::to_string(incomplete_count) + " incomplete frame(s) for this session)");
      }
      const auto width = arena_image->GetWidth();
      const auto height = arena_image->GetHeight();
      const auto bits_per_pixel = arena_image->GetBitsPerPixel();
      if (bits_per_pixel != 24) {
        throw std::runtime_error(
                "Arena returned " + std::to_string(bits_per_pixel) +
                " bits per pixel while BGR8 was configured");
      }
      const auto row_bytes = width * (bits_per_pixel / 8);
      const auto row_stride = row_bytes + arena_image->GetPaddingX();
      const auto required_bytes = row_stride * height + arena_image->GetPaddingY();
      if (arena_image->GetSizeFilled() < required_bytes) {
        const auto incomplete_count = incomplete_frames_.fetch_add(1) + 1;
        throw std::runtime_error(
                "Arena image frame " + std::to_string(arena_image->GetFrameId()) +
                " is smaller than its declared layout: received " +
                std::to_string(arena_image->GetSizeFilled()) + ", require " +
                std::to_string(required_bytes) + " bytes (" +
                std::to_string(incomplete_count) + " incomplete frame(s) for this session)");
      }
      cv::Mat image(
        static_cast<int>(height), static_cast<int>(width), CV_8UC3,
        const_cast<std::uint8_t *>(arena_image->GetData()), row_stride);
      const cv::Mat owned = image.clone();
      device_->RequeueBuffer(arena_image);
      arena_image = nullptr;
      Arena::ExecuteNode(device_->GetNodeMap(), "TransferStop");
      transfer_started = false;
      endpoint_->publish(owned, stamp, true);
      frames_.fetch_add(1);
      set_last_error("");
    } catch (...) {
      const auto error = current_exception_message();
      if (arena_image) {
        try {device_->RequeueBuffer(arena_image);} catch (...) {}
      }
      if (transfer_started) {
        try {Arena::ExecuteNode(device_->GetNodeMap(), "TransferStop");} catch (...) {}
      }
      errors_.fetch_add(1);
      set_last_error(error);
      RCLCPP_ERROR(
        logger_, "Acquisition %lu failed for %s: %s", command.sequence,
        assignment_.sensor_id.c_str(), error.c_str());
    }
  }

  Arena::IDevice * device_;
  Assignment assignment_;
  NetworkConfig network_;
  LucidConfig config_;
  std::unique_ptr<CameraEndpoint> endpoint_;
  rclcpp::Node * node_;
  std::string model_;
  std::string mac_;
  std::string address_;
  std::string mode_;
  rclcpp::Logger logger_;
  mutable std::mutex device_mutex_;
  mutable std::mutex capture_mutex_;
  mutable std::mutex state_mutex_;
  std::condition_variable capture_cv_;
  std::optional<CaptureCommand> pending_;
  bool capture_busy_{false};
  std::thread worker_;
  std::chrono::steady_clock::time_point next_preview_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<bool> ready_{false};
  std::atomic<bool> is_master_{false};
  std::optional<bool> configured_master_;
  std::atomic<std::uint64_t> frames_{0};
  std::atomic<std::uint64_t> errors_{0};
  std::atomic<std::uint64_t> incomplete_frames_{0};
  std::string ptp_status_;
  std::string last_error_;
};

}  // namespace

class LucidProvider : public rclcpp::Node
{
  struct InitializationJob
  {
    Arena::DeviceInfo info;
    Assignment assignment;
    NetworkConfig network;
  };

public:
  LucidProvider()
  : rclcpp::Node("lucid_provider", "/vixel/providers/lucid")
  {
    const auto machine_file = declare_parameter<std::string>(
      "machine_file", "/etc/vixel/machine.yaml");
    config_ = load_lucid_config(machine_file);
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    observation_publisher_ = create_publisher<vixel_interfaces::msg::SensorObservationArray>(
      "observations", state_qos);
    status_publisher_ = create_publisher<vixel_interfaces::msg::SensorArray>("status", state_qos);
    group_publisher_ = create_publisher<vixel_interfaces::msg::SyncGroupArray>(
      "group_status", state_qos);
    assignment_subscription_ = create_subscription<vixel_interfaces::msg::ProviderAssignmentArray>(
      "assignments", state_qos,
      std::bind(&LucidProvider::assignments_callback, this, std::placeholders::_1));
    provision_service_ = create_service<vixel_interfaces::srv::ProvisionSensor>(
      "provision", std::bind(
        &LucidProvider::provision_callback, this, std::placeholders::_1, std::placeholders::_2));
    capture_service_ = create_service<vixel_interfaces::srv::ProviderCapture>(
      "capture", std::bind(
        &LucidProvider::capture_callback, this, std::placeholders::_1, std::placeholders::_2));
    system_ = Arena::OpenSystem();
    const auto worker_count = std::max<std::size_t>(1, config_.networks.size());
    initialization_workers_.reserve(worker_count);
    for (std::size_t index = 0; index < worker_count; ++index) {
      initialization_workers_.emplace_back(&LucidProvider::initialization_loop, this);
    }
    discovery_timer_ = create_wall_timer(
      std::chrono::milliseconds(config_.discovery_period_ms),
      std::bind(&LucidProvider::refresh_devices, this));
    preview_timer_ = create_wall_timer(50ms, std::bind(&LucidProvider::preview_tick, this));
    status_timer_ = create_wall_timer(1s, std::bind(&LucidProvider::publish_status, this));
    RCLCPP_INFO(get_logger(), "LUCID provider started with machine config %s", machine_file.c_str());
    refresh_devices();
  }

  ~LucidProvider() override
  {
    discovery_timer_.reset();
    preview_timer_.reset();
    status_timer_.reset();
    {
      std::lock_guard<std::mutex> queue_lock(initialization_mutex_);
      shutting_down_ = true;
    }
    initialization_cv_.notify_all();
    for (auto & worker : initialization_workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    destroy_all_sessions();
    if (system_) {
      std::lock_guard<std::mutex> arena_lock(arena_mutex_);
      try {Arena::CloseSystem(system_);} catch (...) {}
      system_ = nullptr;
    }
  }

private:
  bool model_allowed(const std::string & model) const
  {
    if (config_.model_allowlist.empty()) {
      return true;
    }
    return std::any_of(
      config_.model_allowlist.begin(), config_.model_allowlist.end(),
      [&model](const std::string & allowed) {return model.find(allowed) != std::string::npos;});
  }

  std::optional<Arena::InterfaceInfo> arena_interface_for(const NetworkConfig & network)
  {
    const auto interfaces = system_->GetInterfaces();
    for (auto candidate : interfaces) {
      if (!network.interface_mac.empty() &&
        candidate.MacAddress() == mac_integer(network.interface_mac))
      {
        return candidate;
      }
      const auto slash = network.host_cidr.find('/');
      if (std::string(candidate.IpAddressStr().c_str()) == network.host_cidr.substr(0, slash)) {
        return candidate;
      }
    }
    return std::nullopt;
  }

  std::vector<Arena::DeviceInfo> update_all_interfaces(std::uint64_t timeout_ms)
  {
    const bool trace = first_discovery_;
    const auto scan_started = std::chrono::steady_clock::now();
    if (trace) {
      RCLCPP_INFO(get_logger(), "Starting bounded Arena interface discovery");
    }
    const auto interfaces = system_->GetInterfaces();
    std::map<std::string, Arena::DeviceInfo> discovered;
    for (auto interface : interfaces) {
      const auto interface_started = std::chrono::steady_clock::now();
      if (trace) {
        RCLCPP_INFO(
          get_logger(), "Discovering Arena interface %s (%s)",
          interface.IpAddressStr().c_str(), interface.MacAddressStr().c_str());
      }
      try {
        system_->UpdateDevices(interface, timeout_ms);
        for (auto info : system_->GetDevices()) {
          discovered[std::string(info.SerialNumber().c_str())] = info;
        }
      } catch (...) {
        const auto error = current_exception_message();
        RCLCPP_DEBUG(
          get_logger(), "Discovery failed on Arena interface %s: %s",
          interface.IpAddressStr().c_str(), error.c_str());
      }
      if (trace) {
        RCLCPP_INFO(
          get_logger(), "Arena interface %s completed in %.3fs",
          interface.IpAddressStr().c_str(),
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - interface_started).count());
      }
    }
    if (trace) {
      RCLCPP_INFO(
        get_logger(), "Arena interface discovery completed in %.3fs across %zu interfaces",
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - scan_started).count(), interfaces.size());
      first_discovery_ = false;
    }
    std::vector<Arena::DeviceInfo> result;
    result.reserve(discovered.size());
    for (auto & item : discovered) {
      result.push_back(item.second);
    }
    return result;
  }

  void update_network_interface(const NetworkConfig & network, std::uint64_t timeout_ms)
  {
    const auto interface = arena_interface_for(network);
    if (!interface) {
      throw std::runtime_error(
              "Arena cannot find host interface " + network.interface + " (" +
              network.interface_mac + ")");
    }
    system_->UpdateDevices(*interface, timeout_ms);
  }

  std::string interface_for(const Arena::DeviceInfo & info)
  {
    try {
      auto * interface_map = system_->GetTLInterfaceNodeMap(info);
      try {
        const auto address = static_cast<std::uint32_t>(
          Arena::GetNodeValue<std::int64_t>(interface_map, "GevInterfaceIPAddress"));
        const std::set<std::string> candidates{
          ipv4_string(address), ipv4_string(byte_swap_ipv4(address))};
        for (const auto & item : config_.networks) {
          const auto slash = item.second.host_cidr.find('/');
          if (candidates.count(item.second.host_cidr.substr(0, slash)) != 0) {
            return item.second.interface;
          }
        }
      } catch (...) {
      }
      try {
        const auto interface_mac = static_cast<std::uint64_t>(
          Arena::GetNodeValue<std::int64_t>(interface_map, "GevInterfaceMACAddress"));
        for (const auto & item : config_.networks) {
          if (!item.second.interface_mac.empty() &&
            mac_integer(item.second.interface_mac) == interface_mac)
          {
            return item.second.interface;
          }
        }
      } catch (...) {
      }
    } catch (...) {
    }
    return "";
  }

  void assignments_callback(
    const vixel_interfaces::msg::ProviderAssignmentArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<std::string, Assignment> updated;
    for (const auto & assignment : message->assignments) {
      updated[assignment.sensor_id] = assignment;
    }
    assignments_ = std::move(updated);
    reconcile_sessions();
  }

  void refresh_devices()
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return;
    }
    try {
      device_infos_.clear();
      device_interfaces_.clear();
      vixel_interfaces::msg::SensorObservationArray observations;
      observations.header.stamp = now();
      std::set<std::string> detected_serials;
      {
        std::lock_guard<std::mutex> arena_lock(arena_mutex_);
        auto infos = update_all_interfaces(100);
        for (auto & info : infos) {
          const std::string serial(info.SerialNumber().c_str());
          const std::string model(info.ModelName().c_str());
          if (!model_allowed(model)) {
            continue;
          }
          detected_serials.insert(serial);
          device_infos_[serial] = info;
          vixel_interfaces::msg::SensorObservation observation;
          observation.stamp = observations.header.stamp;
          observation.provider = "lucid";
          observation.candidate_id = "lucid_" + serial;
          observation.kind = "camera";
          observation.vendor = info.VendorName().c_str();
          observation.model = model;
          observation.serial = serial;
          observation.mac_address = info.MacAddressStr().c_str();
          observation.transport = "gige_vision";
          observation.interface_name = interface_for(info);
          device_interfaces_[serial] = observation.interface_name;
          observation.current_address = info.IpAddressStr().c_str();
          observation.capabilities = {"image", "jpeg_preview", "ptp_action_capture"};
          observations.observations.push_back(observation);
        }
      }
      observation_publisher_->publish(observations);
      for (auto iterator = sessions_.begin(); iterator != sessions_.end();) {
        if (detected_serials.count(iterator->second->serial()) != 0 &&
          assignments_.count(iterator->first) != 0)
        {
          ++iterator;
          continue;
        }
        const auto sensor_id = iterator->first;
        ++iterator;
        destroy_session(sensor_id);
      }
      reconcile_sessions();
      last_system_error_.clear();
    } catch (...) {
      last_system_error_ = current_exception_message();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "LUCID discovery failed: %s",
        last_system_error_.c_str());
    }
  }

  void reconcile_sessions()
  {
    for (auto iterator = sessions_.begin(); iterator != sessions_.end();) {
      const auto assigned = assignments_.find(iterator->first);
      const auto observed_interface = device_interfaces_.find(iterator->second->serial());
      const auto assigned_network = assigned == assignments_.end() ? config_.networks.end() :
        config_.networks.find(assigned->second.network_id);
      if (assigned == assignments_.end() || !assigned->second.enabled ||
        assigned->second.operating_mode == "idle" ||
        assigned->second.operating_mode != iterator->second->mode() ||
        assigned->second.network_id != iterator->second->network_id() ||
        assigned_network == config_.networks.end() ||
        observed_interface == device_interfaces_.end() ||
        observed_interface->second != assigned_network->second.interface)
      {
        const auto sensor_id = iterator->first;
        ++iterator;
        destroy_session(sensor_id);
      } else {
        ++iterator;
      }
    }
    for (const auto & item : assignments_) {
      const auto & assignment = item.second;
      if (!assignment.enabled || assignment.operating_mode == "idle" ||
        sessions_.count(item.first) != 0)
      {
        continue;
      }
      const auto info = device_infos_.find(assignment.serial);
      const auto network = config_.networks.find(assignment.network_id);
      if (info == device_infos_.end() || network == config_.networks.end()) {
        continue;
      }
      const auto observed_interface = device_interfaces_.find(assignment.serial);
      if (observed_interface == device_interfaces_.end() ||
        observed_interface->second != network->second.interface)
      {
        continue;
      }
      if (initializing_.count(assignment.sensor_id) != 0) {
        continue;
      }
      const auto retry = initialization_retry_after_.find(assignment.sensor_id);
      if (retry != initialization_retry_after_.end() &&
        std::chrono::steady_clock::now() < retry->second)
      {
        continue;
      }
      initializing_.insert(assignment.sensor_id);
      {
        std::lock_guard<std::mutex> queue_lock(initialization_mutex_);
        initialization_queue_.push_back({info->second, assignment, network->second});
      }
      initialization_cv_.notify_one();
      RCLCPP_INFO(
        get_logger(), "Queued initialization for %s (%s)",
        assignment.sensor_id.c_str(), assignment.serial.c_str());
    }
    configure_capture_groups();
    publish_status();
  }

  void initialization_loop()
  {
    while (true) {
      InitializationJob job;
      {
        std::unique_lock<std::mutex> queue_lock(initialization_mutex_);
        initialization_cv_.wait(queue_lock, [this]() {
          return shutting_down_ || !initialization_queue_.empty();
        });
        if (shutting_down_ && initialization_queue_.empty()) {
          return;
        }
        job = std::move(initialization_queue_.front());
        initialization_queue_.pop_front();
      }
      Arena::IDevice * device = nullptr;
      std::unique_ptr<CameraSession> session;
      std::string error;
      const auto opening_started = std::chrono::steady_clock::now();
      try {
        auto info = job.info;
        if (!job.assignment.assigned_address.empty() &&
          job.assignment.assigned_address != std::string(info.IpAddressStr().c_str()))
        {
          RCLCPP_INFO(
            get_logger(), "Restoring %s address from %s to %s",
            job.assignment.sensor_id.c_str(), info.IpAddressStr().c_str(),
            job.assignment.assigned_address.c_str());
          info = provision_info(info, job.assignment.assigned_address, job.network);
        }
        {
          std::lock_guard<std::mutex> arena_lock(arena_mutex_);
          update_network_interface(job.network, 100);
          auto current_infos = system_->GetDevices();
          const auto current = std::find_if(
            current_infos.begin(), current_infos.end(), [&](auto & candidate) {
              return std::string(candidate.SerialNumber().c_str()) == job.assignment.serial;
            });
          if (current == current_infos.end()) {
            throw std::runtime_error("camera disappeared from its assigned interface");
          }
          info = *current;
          device = system_->CreateDevice(info);
        }
        RCLCPP_INFO(
          get_logger(), "Opened %s in %.3fs; configuring camera",
          job.assignment.sensor_id.c_str(),
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - opening_started).count());
        session = std::make_unique<CameraSession>(
          device, job.assignment, job.network, config_, this,
          std::string(info.ModelName().c_str()),
          std::string(info.MacAddressStr().c_str()),
          std::string(info.IpAddressStr().c_str()), get_logger());
        session->initialize();
      } catch (...) {
        error = current_exception_message();
      }
      bool keep_session = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto assigned = assignments_.find(job.assignment.sensor_id);
        keep_session = session && error.empty() && assigned != assignments_.end() &&
          assigned->second.enabled &&
          assigned->second.operating_mode == job.assignment.operating_mode &&
          assigned->second.network_id == job.assignment.network_id &&
          sessions_.count(job.assignment.sensor_id) == 0;
        if (keep_session) {
          sessions_[job.assignment.sensor_id] = std::move(session);
          initialization_errors_.erase(job.assignment.sensor_id);
          initialization_retry_after_.erase(job.assignment.sensor_id);
          configure_capture_groups();
        } else if (!error.empty()) {
          initialization_errors_[job.assignment.sensor_id] = error;
          initialization_retry_after_[job.assignment.sensor_id] =
            std::chrono::steady_clock::now() + 2s;
        }
        initializing_.erase(job.assignment.sensor_id);
      }
      if (!keep_session && device) {
        if (session) {
          session->shutdown();
        }
        std::lock_guard<std::mutex> arena_lock(arena_mutex_);
        try {system_->DestroyDevice(device);} catch (...) {}
      }
      if (keep_session) {
        RCLCPP_INFO(
          get_logger(), "Started %s (%s) at %s in %s mode",
          job.assignment.sensor_id.c_str(), job.assignment.serial.c_str(),
          job.assignment.assigned_address.c_str(), job.assignment.operating_mode.c_str());
      } else if (!error.empty()) {
        RCLCPP_ERROR(
          get_logger(), "Failed to start %s: %s",
          job.assignment.sensor_id.c_str(), error.c_str());
      }
      publish_status();
    }
  }

  void configure_capture_groups()
  {
    std::map<std::string, std::vector<CameraSession *>> groups;
    for (auto & item : sessions_) {
      if (item.second->mode() == "capture" && !item.second->assignment().sync_group.empty()) {
        groups[item.second->assignment().sync_group].push_back(item.second.get());
      }
    }
    for (auto & item : groups) {
      auto & members = item.second;
      std::sort(members.begin(), members.end(), [](const auto * left, const auto * right) {
        return left->sensor_id() < right->sensor_id();
      });
      std::string master = members.front()->sensor_id();
      const auto preferred = members.front()->assignment().preferred_master_id;
      if (!preferred.empty() && std::any_of(
          members.begin(), members.end(), [&preferred](const auto * session) {
            return session->sensor_id() == preferred;
          }))
      {
        master = preferred;
      }
      for (auto * session : members) {
        try {session->configure_ptp(session->sensor_id() == master);} catch (...) {
          const auto error = current_exception_message();
          RCLCPP_ERROR(
            get_logger(), "PTP setup failed for %s: %s",
            session->sensor_id().c_str(), error.c_str());
        }
      }
    }
  }

  void preview_tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto tick = std::chrono::steady_clock::now();
    const auto sequence = ++preview_sequence_;
    auto transfer_at = tick;
    for (auto & item : sessions_) {
      auto & session = *item.second;
      if (!session.preview_due(tick)) {
        continue;
      }
      CaptureCommand command;
      command.sequence = sequence;
      command.transfer_at = transfer_at;
      command.software_trigger = true;
      if (!session.queue(std::move(command))) {
        RCLCPP_DEBUG(get_logger(), "Preview skipped for busy sensor %s", item.first.c_str());
      } else {
        transfer_at += session.transfer_slot();
      }
    }
  }

  void publish_status()
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return;
    }
    for (auto & item : sessions_) {
      try {item.second->refresh_ptp();} catch (...) {}
    }
    vixel_interfaces::msg::SensorArray status;
    status.header.stamp = now();
    status.generation = ++status_generation_;
    for (const auto & assignment : assignments_) {
      const auto session = sessions_.find(assignment.first);
      if (session != sessions_.end()) {
        status.sensors.push_back(session->second->status());
      } else {
        Sensor sensor;
        sensor.stamp = status.header.stamp;
        sensor.sensor_id = assignment.first;
        sensor.provider = "lucid";
        sensor.kind = "camera";
        sensor.vendor = "LUCID";
        sensor.serial = assignment.second.serial;
        sensor.mac_address = assignment.second.mac_address;
        sensor.enrolled = true;
        sensor.enabled = assignment.second.enabled;
        const auto observed_interface = device_interfaces_.find(assignment.second.serial);
        const auto network = config_.networks.find(assignment.second.network_id);
        const bool placement_matches = observed_interface != device_interfaces_.end() &&
          network != config_.networks.end() &&
          observed_interface->second == network->second.interface;
        sensor.online = device_infos_.count(assignment.second.serial) != 0 && placement_matches;
        if (!placement_matches && observed_interface != device_interfaces_.end()) {
          sensor.lifecycle_state = "placement_conflict";
          sensor.status_detail = "Camera was detected on a different managed interface";
        } else {
          sensor.lifecycle_state = assignment.second.operating_mode == "idle" ? "idle" :
            (initializing_.count(assignment.first) != 0 ? "configuring" : "offline");
          sensor.status_detail = initializing_.count(assignment.first) != 0 ?
            "Opening and configuring camera" : "";
        }
        sensor.topic_base = "/vixel/sensors/" + assignment.first;
        sensor.network_id = assignment.second.network_id;
        sensor.assigned_address = assignment.second.assigned_address;
        sensor.sync_group = assignment.second.sync_group;
        sensor.operating_mode = assignment.second.operating_mode;
        sensor.capabilities = {"image", "jpeg_preview", "ptp_action_capture"};
        const auto initialization_error = initialization_errors_.find(assignment.first);
        if (initialization_error != initialization_errors_.end()) {
          sensor.last_error = initialization_error->second;
        }
        status.sensors.push_back(sensor);
      }
    }
    status_publisher_->publish(status);
    publish_group_status(status.header.stamp);
  }

  void publish_group_status(const builtin_interfaces::msg::Time & stamp)
  {
    std::map<std::string, SyncGroup> groups;
    for (const auto & assignment : assignments_) {
      if (assignment.second.sync_group.empty()) {
        continue;
      }
      auto & group = groups[assignment.second.sync_group];
      group.stamp = stamp;
      group.group_id = assignment.second.sync_group;
      group.provider = "lucid";
      group.operating_mode = assignment.second.operating_mode;
      group.preview_rate_hz = assignment.second.preview_rate_hz;
      group.missing_policy = assignment.second.group_missing_policy;
      group.preferred_master_id = assignment.second.preferred_master_id;
      group.member_ids.push_back(assignment.first);
      const auto session = sessions_.find(assignment.first);
      if (session != sessions_.end() && session->second->ready()) {
        group.online_member_ids.push_back(assignment.first);
      } else {
        group.missing_member_ids.push_back(assignment.first);
      }
    }
    vixel_interfaces::msg::SyncGroupArray message;
    message.header.stamp = stamp;
    message.generation = status_generation_;
    for (auto & item : groups) {
      auto & group = item.second;
      group.ready = group.operating_mode == "idle" || group.missing_member_ids.empty() ||
        (group.missing_policy == "degraded" && !group.online_member_ids.empty());
      message.groups.push_back(group);
    }
    group_publisher_->publish(message);
  }

  void provision_callback(
    const std::shared_ptr<vixel_interfaces::srv::ProvisionSensor::Request> request,
    std::shared_ptr<vixel_interfaces::srv::ProvisionSensor::Response> response)
  {
    try {
      std::optional<Arena::DeviceInfo> selected_info;
      NetworkConfig selected_network;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto info = device_infos_.find(request->serial);
        const auto network = config_.networks.find(request->network_id);
        if (info == device_infos_.end()) {
          throw std::runtime_error("camera is no longer discoverable");
        }
        if (network == config_.networks.end()) {
          throw std::runtime_error("unknown managed network " + request->network_id);
        }
        for (auto & other : device_infos_) {
          if (other.first != request->serial &&
            std::string(other.second.IpAddressStr().c_str()) == request->target_address)
          {
            throw std::runtime_error("target address is already used by camera " + other.first);
          }
        }
        selected_info = info->second;
        selected_network = network->second;
      }
      provision_info(
        *selected_info, request->target_address, selected_network,
        request->subnet_mask, request->gateway);
      response->success = true;
      response->message = "persistent camera address configured and verified";
      response->current_address = request->target_address;
    } catch (...) {
      response->success = false;
      response->message = current_exception_message();
    }
  }

  Arena::DeviceInfo provision_info(
    Arena::DeviceInfo info, const std::string & address, const NetworkConfig & network,
    std::string subnet_mask = "", std::string gateway = "0.0.0.0")
  {
    std::lock_guard<std::mutex> arena_lock(arena_mutex_);
    if (subnet_mask.empty()) {
      const auto slash = network.host_cidr.find('/');
      const int prefix = slash == std::string::npos ? 24 : std::stoi(network.host_cidr.substr(slash + 1));
      const std::uint32_t mask = prefix == 0 ? 0U : 0xFFFFFFFFU << (32 - prefix);
      subnet_mask = ipv4_string(mask);
    }
    system_->ForceIp(
      info.MacAddress(), ipv4_integer(address), ipv4_integer(subnet_mask),
      ipv4_integer(gateway));
    std::this_thread::sleep_for(150ms);
    update_network_interface(network, 200);
    auto current_infos = system_->GetDevices();
    const std::string serial(info.SerialNumber().c_str());
    auto current = std::find_if(current_infos.begin(), current_infos.end(), [&serial](auto & candidate) {
      return std::string(candidate.SerialNumber().c_str()) == serial;
    });
    if (current == current_infos.end()) {
      throw std::runtime_error("camera did not rediscover after ForceIP");
    }
    Arena::IDevice * device = system_->CreateDevice(*current);
    try {
      auto * map = device->GetNodeMap();
      Arena::SetNodeValue<std::int64_t>(
        map, "GevPersistentIPAddress", static_cast<std::int64_t>(ipv4_integer(address)));
      Arena::SetNodeValue<std::int64_t>(
        map, "GevPersistentSubnetMask", static_cast<std::int64_t>(ipv4_integer(subnet_mask)));
      Arena::SetNodeValue<std::int64_t>(
        map, "GevPersistentDefaultGateway", static_cast<std::int64_t>(ipv4_integer(gateway)));
      Arena::SetNodeValue<bool>(map, "GevCurrentIPConfigurationPersistentIP", true);
      Arena::SetNodeValue<bool>(map, "GevCurrentIPConfigurationDHCP", false);
      const auto read_address = Arena::GetNodeValue<std::int64_t>(map, "GevPersistentIPAddress");
      const auto persistent = Arena::GetNodeValue<bool>(
        map, "GevCurrentIPConfigurationPersistentIP");
      if (!persistent || read_address != static_cast<std::int64_t>(ipv4_integer(address))) {
        throw std::runtime_error("camera did not retain persistent IP settings");
      }
      system_->DestroyDevice(device);
    } catch (...) {
      try {system_->DestroyDevice(device);} catch (...) {}
      throw;
    }
    update_network_interface(network, 200);
    auto verified = system_->GetDevices();
    const auto match = std::find_if(verified.begin(), verified.end(), [&](auto & candidate) {
      return std::string(candidate.SerialNumber().c_str()) == serial &&
             std::string(candidate.IpAddressStr().c_str()) == address;
    });
    if (match == verified.end()) {
      throw std::runtime_error("camera address verification failed after persistent configuration");
    }
    return *match;
  }

  void capture_callback(
    const std::shared_ptr<vixel_interfaces::srv::ProviderCapture::Request> request,
    std::shared_ptr<vixel_interfaces::srv::ProviderCapture::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      {
        std::lock_guard<std::mutex> state_lock(capture_state_mutex_);
        if (capture_active_[request->group_id]) {
          throw std::runtime_error("a capture is already active for this group");
        }
      }
      std::vector<CameraSession *> participating;
      for (const auto & sensor_id : request->member_ids) {
        const auto session = sessions_.find(sensor_id);
        if (session == sessions_.end() || session->second->mode() != "capture" ||
          !session->second->can_capture())
        {
          response->missing_sensor_ids.push_back(sensor_id);
        } else {
          participating.push_back(session->second.get());
          response->participating_sensor_ids.push_back(sensor_id);
        }
      }
      if (participating.empty()) {
        throw std::runtime_error("no capture-ready group members are online");
      }
      if (request->missing_policy == "strict" && !response->missing_sensor_ids.empty()) {
        throw std::runtime_error("strict group has missing or unready members");
      }
      auto master = std::find_if(participating.begin(), participating.end(), [](auto * session) {
        return session->is_master();
      });
      if (master == participating.end()) {
        throw std::runtime_error("group PTP master is not ready");
      }
      const auto sequence = ++capture_sequence_;
      const auto capture_id = request->request_id.empty() ?
        request->group_id + "-" + std::to_string(sequence) : request->request_id;
      const std::int64_t ptp_now = (*master)->latch_ptp_time();
      const std::int64_t scheduled = ptp_now +
        static_cast<std::int64_t>(config_.schedule_lead_ms) * 1000000LL;
      auto * transport = system_->GetTLSystemNodeMap();
      Arena::SetNodeValue<std::int64_t>(transport, "ActionCommandDeviceKey", config_.action_device_key);
      Arena::SetNodeValue<std::int64_t>(
        transport, "ActionCommandGroupKey",
        action_group_key(request->group_id, config_.action_group_key));
      Arena::SetNodeValue<std::int64_t>(transport, "ActionCommandGroupMask", config_.action_group_mask);
      Arena::SetNodeValue<std::int64_t>(transport, "ActionCommandTargetIP", 0xFFFFFFFFLL);
      Arena::SetNodeValue<std::int64_t>(transport, "ActionCommandExecuteTime", scheduled);
      Arena::ExecuteNode(transport, "ActionCommandFireCommand");
      {
        std::lock_guard<std::mutex> state_lock(capture_state_mutex_);
        capture_active_[request->group_id] = true;
      }
      auto batch = std::make_shared<CaptureBatch>(participating.size(), [this, group=request->group_id]() {
        std::lock_guard<std::mutex> callback_lock(capture_state_mutex_);
        capture_active_[group] = false;
      });
      const auto base_time = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(config_.schedule_lead_ms);
      std::map<std::string, int> network_slots;
      std::sort(participating.begin(), participating.end(), [](auto * left, auto * right) {
        return left->sensor_id() < right->sensor_id();
      });
      for (auto * session : participating) {
        const auto network = config_.networks.at(session->network_id());
        const int slot = network_slots[session->network_id()]++;
        CaptureCommand command;
        command.sequence = sequence;
        command.stamp = ptp_to_ros_time(scheduled);
        command.transfer_at = base_time + std::chrono::milliseconds(network.transfer_slot_ms * slot);
        command.batch = batch;
        if (!session->queue(std::move(command))) {
          throw std::runtime_error("failed to queue capture for " + session->sensor_id());
        }
      }
      response->success = true;
      response->message = response->missing_sensor_ids.empty() ?
        "synchronized capture scheduled" : "degraded capture scheduled for available members";
      response->scheduled_time = ptp_to_ros_time(scheduled);
      response->capture_id = capture_id;
    } catch (...) {
      response->success = false;
      response->message = current_exception_message();
    }
  }

  void destroy_session(const std::string & sensor_id)
  {
    const auto found = sessions_.find(sensor_id);
    if (found == sessions_.end()) {
      return;
    }
    auto * device = found->second->device();
    found->second->shutdown();
    {
      std::lock_guard<std::mutex> arena_lock(arena_mutex_);
      try {system_->DestroyDevice(device);} catch (...) {}
    }
    sessions_.erase(found);
    RCLCPP_INFO(get_logger(), "Stopped %s", sensor_id.c_str());
  }

  void destroy_all_sessions()
  {
    while (!sessions_.empty()) {
      destroy_session(sessions_.begin()->first);
    }
  }

  LucidConfig config_;
  Arena::ISystem * system_{nullptr};
  std::mutex mutex_;
  std::mutex arena_mutex_;
  std::mutex capture_state_mutex_;
  std::mutex initialization_mutex_;
  std::condition_variable initialization_cv_;
  std::deque<InitializationJob> initialization_queue_;
  std::set<std::string> initializing_;
  std::map<std::string, std::chrono::steady_clock::time_point> initialization_retry_after_;
  std::map<std::string, std::string> initialization_errors_;
  std::vector<std::thread> initialization_workers_;
  bool shutting_down_{false};
  bool first_discovery_{true};
  std::map<std::string, Arena::DeviceInfo> device_infos_;
  std::map<std::string, std::string> device_interfaces_;
  std::map<std::string, Assignment> assignments_;
  std::map<std::string, std::unique_ptr<CameraSession>> sessions_;
  std::map<std::string, bool> capture_active_;
  std::string last_system_error_;
  std::uint64_t preview_sequence_{0};
  std::uint64_t capture_sequence_{0};
  std::uint64_t status_generation_{0};
  rclcpp::Publisher<vixel_interfaces::msg::SensorObservationArray>::SharedPtr observation_publisher_;
  rclcpp::Publisher<vixel_interfaces::msg::SensorArray>::SharedPtr status_publisher_;
  rclcpp::Publisher<vixel_interfaces::msg::SyncGroupArray>::SharedPtr group_publisher_;
  rclcpp::Subscription<vixel_interfaces::msg::ProviderAssignmentArray>::SharedPtr assignment_subscription_;
  rclcpp::Service<vixel_interfaces::srv::ProvisionSensor>::SharedPtr provision_service_;
  rclcpp::Service<vixel_interfaces::srv::ProviderCapture>::SharedPtr capture_service_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr preview_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace vixel_lucid

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto * runtime_dir = std::getenv("XDG_RUNTIME_DIR");
  const std::string lock_path = runtime_dir && runtime_dir[0] != '\0' ?
    std::string(runtime_dir) + "/vixel-lucid-provider.lock" :
    "/tmp/vixel-lucid-provider-" + std::to_string(::getuid()) + ".lock";
  const int lock_fd = ::open(
    lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (lock_fd < 0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lucid_provider"), "Cannot open instance lock %s: %s",
      lock_path.c_str(), std::strerror(errno));
    rclcpp::shutdown();
    return 1;
  }
  if (::flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lucid_provider"),
      "Another Vixel LUCID provider is already running (lock %s)", lock_path.c_str());
    ::close(lock_fd);
    rclcpp::shutdown();
    return 1;
  }
  ::ftruncate(lock_fd, 0);
  const auto pid_text = std::to_string(::getpid()) + "\n";
  static_cast<void>(::write(lock_fd, pid_text.data(), pid_text.size()));

  int result = 0;
  try {
    auto node = std::make_shared<vixel_lucid::LucidProvider>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (...) {
    const auto error = vixel_lucid::current_exception_message();
    RCLCPP_FATAL(rclcpp::get_logger("lucid_provider"), "%s", error.c_str());
    result = 1;
  }
  rclcpp::shutdown();
  ::close(lock_fd);
  return result;
}
