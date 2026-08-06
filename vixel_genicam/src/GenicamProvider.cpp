#include "vixel_genicam/GenicamConfig.hpp"

#include <arv.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vixel_interfaces/msg/camera_feature.hpp>
#include <vixel_interfaces/msg/camera_timing.hpp>
#include <vixel_interfaces/msg/capture_frame_chunk.hpp>
#include <vixel_interfaces/msg/provider_assignment.hpp>
#include <vixel_interfaces/msg/provider_assignment_array.hpp>
#include <vixel_interfaces/msg/sensor.hpp>
#include <vixel_interfaces/msg/sensor_array.hpp>
#include <vixel_interfaces/msg/sensor_observation.hpp>
#include <vixel_interfaces/msg/sensor_observation_array.hpp>
#include <vixel_interfaces/msg/sync_group.hpp>
#include <vixel_interfaces/msg/sync_group_array.hpp>
#include <vixel_interfaces/srv/get_camera_features.hpp>
#include <vixel_interfaces/srv/provider_capture.hpp>
#include <vixel_interfaces/srv/provision_sensor.hpp>
#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace vixel_genicam
{
namespace
{
using Assignment = vixel_interfaces::msg::ProviderAssignment;
using Sensor = vixel_interfaces::msg::Sensor;
using SyncGroup = vixel_interfaces::msg::SyncGroup;

struct DeviceRecord
{
  std::string device_id;
  std::string physical_id;
  std::string address;
  std::string vendor;
  std::string model;
  std::string serial;
  std::string protocol;
  std::string interface_name;
};

class ScopedSocket
{
public:
  explicit ScopedSocket(int value = -1) : value_(value) {}
  ~ScopedSocket() {if (value_ >= 0) {close(value_);}}
  ScopedSocket(const ScopedSocket &) = delete;
  ScopedSocket & operator=(const ScopedSocket &) = delete;
  int get() const {return value_;}

private:
  int value_;
};

std::string value_or_empty(const char * value)
{
  return value == nullptr ? std::string{} : std::string(value);
}

std::string fixed_text(const std::uint8_t * value, std::size_t size)
{
  std::size_t length = 0;
  while (length < size && value[length] != 0) {++length;}
  std::string result(reinterpret_cast<const char *>(value), length);
  while (!result.empty() && std::isspace(static_cast<unsigned char>(result.back()))) {
    result.pop_back();
  }
  return result;
}

std::vector<DeviceRecord> gvcp_discover(
  const std::string & interface_name, const std::string & source_address,
  std::chrono::milliseconds timeout)
{
  constexpr std::uint16_t gvcp_port = 3956;
  constexpr std::uint16_t discovery_command = 0x0002;
  constexpr std::uint16_t discovery_acknowledge = 0x0003;
  constexpr std::size_t discovery_reply_size = 256;
  static std::atomic<std::uint16_t> request_sequence{1};

  ScopedSocket source_socket(socket(AF_INET, SOCK_DGRAM, 0));
  if (source_socket.get() < 0) {throw std::runtime_error("cannot create GVCP discovery socket");}
  int enabled = 1;
  if (setsockopt(
      source_socket.get(), SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled)) != 0 ||
    setsockopt(
      source_socket.get(), SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled)) != 0)
  {
    throw std::runtime_error("cannot configure GVCP discovery socket");
  }
  sockaddr_in source{};
  source.sin_family = AF_INET;
  source.sin_port = 0;
  if (inet_pton(AF_INET, source_address.c_str(), &source.sin_addr) != 1 ||
    bind(
      source_socket.get(), reinterpret_cast<const sockaddr *>(&source), sizeof(source)) != 0)
  {
    throw std::runtime_error(
            "cannot bind GVCP discovery source " + source_address + " on " + interface_name);
  }
  socklen_t source_size = sizeof(source);
  if (getsockname(
      source_socket.get(), reinterpret_cast<sockaddr *>(&source), &source_size) != 0)
  {
    throw std::runtime_error("cannot determine GVCP discovery source port");
  }

  // Off-subnet cameras broadcast their acknowledgement because they cannot
  // route a unicast response to the host address. A wildcard socket on the
  // same port is therefore required in addition to the source-bound socket.
  ScopedSocket wildcard_socket(socket(AF_INET, SOCK_DGRAM, 0));
  if (wildcard_socket.get() < 0 || setsockopt(
      wildcard_socket.get(), SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled)) != 0)
  {
    throw std::runtime_error("cannot create GVCP wildcard receive socket");
  }
  sockaddr_in wildcard{};
  wildcard.sin_family = AF_INET;
  wildcard.sin_port = source.sin_port;
  wildcard.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(
      wildcard_socket.get(), reinterpret_cast<const sockaddr *>(&wildcard),
      sizeof(wildcard)) != 0)
  {
    throw std::runtime_error("cannot bind GVCP wildcard receive socket");
  }

  std::array<std::uint8_t, 8> request{
    0x42, 0x11,
    static_cast<std::uint8_t>((discovery_command >> 8) & 0xff),
    static_cast<std::uint8_t>(discovery_command & 0xff),
    0x00, 0x00, 0x00, 0x00};
  const auto next_request = request_sequence.fetch_add(1);
  const auto request_id = next_request == 0 ? 1 : next_request;
  request[6] = static_cast<std::uint8_t>((request_id >> 8) & 0xff);
  request[7] = static_cast<std::uint8_t>(request_id & 0xff);
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(gvcp_port);
  destination.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  if (sendto(
      source_socket.get(), request.data(), request.size(), 0,
      reinterpret_cast<const sockaddr *>(&destination), sizeof(destination)) !=
    static_cast<ssize_t>(request.size()))
  {
    throw std::runtime_error(
            "GVCP discovery send failed on " + interface_name + ": " + std::strerror(errno));
  }

  std::map<std::string, DeviceRecord> discovered;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::array<pollfd, 2> sockets{{
    {source_socket.get(), POLLIN, 0}, {wildcard_socket.get(), POLLIN, 0}}};
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (poll(sockets.data(), sockets.size(), std::max(1, static_cast<int>(remaining.count()))) <= 0) {
      break;
    }
    for (auto & candidate : sockets) {
      if ((candidate.revents & POLLIN) == 0) {continue;}
      std::array<std::uint8_t, 1024> reply{};
      const auto length = recv(candidate.fd, reply.data(), reply.size(), 0);
      if (length < static_cast<ssize_t>(discovery_reply_size) ||
        reply[2] != static_cast<std::uint8_t>((discovery_acknowledge >> 8) & 0xff) ||
        reply[3] != static_cast<std::uint8_t>(discovery_acknowledge & 0xff) ||
        reply[6] != request[6] || reply[7] != request[7])
      {
        continue;
      }
      DeviceRecord record;
      std::ostringstream mac;
      mac << std::hex;
      for (std::size_t index = 18; index < 24; ++index) {
        if (index != 18) {mac << ':';}
        mac.width(2);
        mac.fill('0');
        mac << static_cast<unsigned int>(reply[index]);
      }
      record.physical_id = mac.str();
      in_addr address{};
      std::memcpy(&address.s_addr, reply.data() + 44, sizeof(address.s_addr));
      std::array<char, INET_ADDRSTRLEN> address_text{};
      if (inet_ntop(AF_INET, &address, address_text.data(), address_text.size()) == nullptr) {
        continue;
      }
      record.address = address_text.data();
      record.vendor = fixed_text(reply.data() + 80, 32);
      record.model = fixed_text(reply.data() + 112, 32);
      record.serial = fixed_text(reply.data() + 224, 16);
      record.device_id = record.vendor + "-" + record.serial;
      record.protocol = "GigEVision";
      record.interface_name = interface_name;
      if (!record.serial.empty()) {discovered[record.vendor + "\n" + record.serial] = record;}
    }
  }
  std::vector<DeviceRecord> result;
  for (auto & item : discovered) {result.push_back(std::move(item.second));}
  return result;
}

bool feature_writable(ArvCamera * camera, const char * feature)
{
  const auto access = arv_device_get_feature_access_mode(
    arv_camera_get_device(camera), feature);
  return access == ARV_GC_ACCESS_MODE_WO || access == ARV_GC_ACCESS_MODE_RW;
}

std::string error_text(GError * error, const std::string & context)
{
  if (error == nullptr) {
    return context;
  }
  const std::string result = context + ": " + error->message;
  g_error_free(error);
  return result;
}

void throw_on_error(GError * error, const std::string & context)
{
  if (error != nullptr) {
    throw std::runtime_error(error_text(error, context));
  }
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

std::string joined(const std::vector<std::string> & values)
{
  std::ostringstream result;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {result << ", ";}
    result << values[index];
  }
  return result.str();
}

std::string ptp_lock_error(const std::vector<std::string> & sensor_ids)
{
  return "PTP lock not ready for: " + joined(sensor_ids) +
         "; trigger request rejected before dispatch";
}

std::string ptp_lock_status(const std::vector<std::string> & sensor_ids)
{
  return "Waiting for PTP lock: " + joined(sensor_ids) +
         "; trigger requests are disabled";
}

std::string ros_slug(const std::string & value)
{
  std::string result;
  for (const auto character : lower(value)) {
    result.push_back(std::isalnum(static_cast<unsigned char>(character)) ? character : '_');
  }
  while (result.find("__") != std::string::npos) {
    result.replace(result.find("__"), 2, "_");
  }
  while (!result.empty() && result.front() == '_') {result.erase(result.begin());}
  while (!result.empty() && result.back() == '_') {result.pop_back();}
  if (result.empty() || !std::isalpha(static_cast<unsigned char>(result.front()))) {
    result = "camera_" + result;
  }
  return result;
}

std::string normalized_mac(const std::string & raw)
{
  std::string compact;
  for (const auto character : raw) {
    if (std::isxdigit(static_cast<unsigned char>(character))) {
      compact.push_back(static_cast<char>(std::tolower(character)));
    }
  }
  if (compact.size() != 12) {
    return raw;
  }
  std::string result;
  for (std::size_t index = 0; index < compact.size(); index += 2) {
    if (!result.empty()) {result.push_back(':');}
    result.append(compact.substr(index, 2));
  }
  return result;
}

bool interface_has_carrier(const std::string & interface_name)
{
  std::ifstream stream("/sys/class/net/" + interface_name + "/carrier");
  int carrier = 0;
  return stream >> carrier && carrier == 1;
}

bool address_in_cidr(const std::string & address, const std::string & cidr)
{
  const auto slash = cidr.find('/');
  if (slash == std::string::npos) {return false;}
  in_addr candidate{}, network{};
  if (inet_pton(AF_INET, address.c_str(), &candidate) != 1 ||
    inet_pton(AF_INET, cidr.substr(0, slash).c_str(), &network) != 1)
  {
    return false;
  }
  const int prefix = std::stoi(cidr.substr(slash + 1));
  const std::uint32_t mask = prefix == 0 ? 0U : 0xffffffffU << (32 - prefix);
  return (ntohl(candidate.s_addr) & mask) == (ntohl(network.s_addr) & mask);
}

std::string subnet_mask_for_cidr(const std::string & cidr)
{
  const auto slash = cidr.find('/');
  if (slash == std::string::npos) {throw std::runtime_error("host CIDR has no prefix");}
  const int prefix = std::stoi(cidr.substr(slash + 1));
  if (prefix < 0 || prefix > 32) {throw std::runtime_error("invalid host CIDR prefix");}
  const std::uint32_t mask = prefix == 0 ? 0U : 0xffffffffU << (32 - prefix);
  in_addr address{htonl(mask)};
  std::array<char, INET_ADDRSTRLEN> text{};
  if (inet_ntop(AF_INET, &address, text.data(), text.size()) == nullptr) {
    throw std::runtime_error("failed to format subnet mask");
  }
  return text.data();
}

std::string host_address_for_cidr(const std::string & cidr)
{
  const auto slash = cidr.find('/');
  if (slash == std::string::npos) {throw std::runtime_error("host CIDR has no prefix");}
  return cidr.substr(0, slash);
}

std::string broadcast_address_for_cidr(const std::string & cidr)
{
  const auto slash = cidr.find('/');
  if (slash == std::string::npos) {throw std::runtime_error("host CIDR has no prefix");}
  const int prefix = std::stoi(cidr.substr(slash + 1));
  if (prefix < 0 || prefix > 32) {throw std::runtime_error("invalid host CIDR prefix");}
  in_addr host{};
  if (inet_pton(AF_INET, cidr.substr(0, slash).c_str(), &host) != 1) {
    throw std::runtime_error("invalid host CIDR address");
  }
  const std::uint32_t mask = prefix == 0 ? 0U : 0xffffffffU << (32 - prefix);
  in_addr broadcast{htonl((ntohl(host.s_addr) & mask) | ~mask)};
  std::array<char, INET_ADDRSTRLEN> text{};
  if (inet_ntop(AF_INET, &broadcast, text.data(), text.size()) == nullptr) {
    throw std::runtime_error("failed to format broadcast address");
  }
  return text.data();
}

std::array<std::uint8_t, 6> mac_bytes(const std::string & value)
{
  std::string compact;
  for (const auto character : value) {
    if (std::isxdigit(static_cast<unsigned char>(character))) {compact.push_back(character);}
  }
  if (compact.size() != 12) {throw std::runtime_error("invalid camera MAC address " + value);}
  std::array<std::uint8_t, 6> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = static_cast<std::uint8_t>(
      std::stoul(compact.substr(index * 2, 2), nullptr, 16));
  }
  return result;
}

void write_u16(std::uint8_t * destination, std::uint16_t value)
{
  destination[0] = static_cast<std::uint8_t>((value >> 8) & 0xff);
  destination[1] = static_cast<std::uint8_t>(value & 0xff);
}

std::uint16_t read_u16(const std::uint8_t * source)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(source[0]) << 8) |
    static_cast<std::uint16_t>(source[1]));
}

void write_u32(std::uint8_t * destination, std::uint32_t value)
{
  for (int shift = 24, index = 0; shift >= 0; shift -= 8, ++index) {
    destination[index] = static_cast<std::uint8_t>((value >> shift) & 0xff);
  }
}

void write_u64(std::uint8_t * destination, std::uint64_t value)
{
  for (int shift = 56, index = 0; shift >= 0; shift -= 8, ++index) {
    destination[index] = static_cast<std::uint8_t>((value >> shift) & 0xff);
  }
}

std::uint32_t action_group_key(const std::string & group_id)
{
  std::uint32_t value = 2166136261U;
  for (const unsigned char character : group_id) {
    value = (value ^ character) * 16777619U;
  }
  return value == 0 ? 1U : value;
}

constexpr std::uint32_t action_group_mask = 0xffffffffU;

void gvcp_scheduled_action(
  const NetworkConfig & network, std::uint32_t device_key,
  std::uint32_t group_key, std::uint64_t action_time)
{
  constexpr std::uint16_t gvcp_port = 3956;
  constexpr std::uint16_t action_command = 0x0100;
  constexpr std::uint16_t payload_size = 20;
  static std::atomic<std::uint16_t> request_sequence{1};
  std::array<std::uint8_t, 8 + payload_size> packet{};
  packet[0] = 0x42;
  // Scheduled Action bit plus acknowledgement request.
  packet[1] = 0x81;
  write_u16(packet.data() + 2, action_command);
  write_u16(packet.data() + 4, payload_size);
  const auto request_id = request_sequence.fetch_add(1);
  write_u16(packet.data() + 6, request_id == 0 ? 1 : request_id);
  write_u32(packet.data() + 8, device_key);
  write_u32(packet.data() + 12, group_key);
  write_u32(packet.data() + 16, action_group_mask);
  write_u64(packet.data() + 20, action_time);

  ScopedSocket command_socket(socket(AF_INET, SOCK_DGRAM, 0));
  if (command_socket.get() < 0) {
    throw std::runtime_error("cannot create scheduled-action socket");
  }
  int enabled = 1;
  if (setsockopt(
      command_socket.get(), SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled)) != 0)
  {
    throw std::runtime_error("cannot enable scheduled-action broadcast");
  }
  timeval receive_timeout{};
  receive_timeout.tv_usec = 50000;
  if (setsockopt(
      command_socket.get(), SOL_SOCKET, SO_RCVTIMEO,
      &receive_timeout, sizeof(receive_timeout)) != 0)
  {
    throw std::runtime_error("cannot set scheduled-action acknowledgement timeout");
  }
  sockaddr_in source{};
  source.sin_family = AF_INET;
  source.sin_port = 0;
  const auto source_address = host_address_for_cidr(network.host_cidr);
  if (inet_pton(AF_INET, source_address.c_str(), &source.sin_addr) != 1 ||
    bind(
      command_socket.get(), reinterpret_cast<const sockaddr *>(&source),
      sizeof(source)) != 0)
  {
    throw std::runtime_error(
            "cannot bind scheduled-action source " + source_address);
  }
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(gvcp_port);
  // GigE Vision Action Commands use the limited broadcast address. Some
  // cameras reject a subnet-directed broadcast even on an otherwise direct link.
  const std::string target_address{"255.255.255.255"};
  if (inet_pton(AF_INET, target_address.c_str(), &destination.sin_addr) != 1) {
    throw std::runtime_error("invalid scheduled-action target " + target_address);
  }
  if (sendto(
      command_socket.get(), packet.data(), packet.size(), 0,
      reinterpret_cast<const sockaddr *>(&destination), sizeof(destination)) !=
    static_cast<ssize_t>(packet.size()))
  {
    throw std::runtime_error(
            "scheduled-action send failed on " + network.interface + ": " +
            std::strerror(errno));
  }
  std::array<std::uint8_t, 64> acknowledgement{};
  sockaddr_in acknowledgement_source{};
  socklen_t acknowledgement_source_size = sizeof(acknowledgement_source);
  const auto received = recvfrom(
    command_socket.get(), acknowledgement.data(), acknowledgement.size(), 0,
    reinterpret_cast<sockaddr *>(&acknowledgement_source),
    &acknowledgement_source_size);
  if (received < 0) {
    throw std::runtime_error(
            "no ACTION_ACK received on " + network.interface + ": " +
            std::strerror(errno));
  }
  constexpr std::uint16_t action_acknowledge = 0x0101;
  if (received < 8 || read_u16(acknowledgement.data() + 2) != action_acknowledge ||
    read_u16(acknowledgement.data() + 6) != (request_id == 0 ? 1 : request_id))
  {
    throw std::runtime_error("invalid ACTION_ACK received on " + network.interface);
  }
  const auto status = read_u16(acknowledgement.data());
  if (status != 0) {
    std::ostringstream message;
    message << "camera rejected scheduled action on " << network.interface <<
      " with GVCP status 0x" << std::hex << status;
    throw std::runtime_error(message.str());
  }
}

void write_ipv4(std::uint8_t * destination, const std::string & value)
{
  in_addr address{};
  if (inet_pton(AF_INET, value.c_str(), &address) != 1) {
    throw std::runtime_error("invalid IPv4 address " + value);
  }
  std::memcpy(destination, &address.s_addr, sizeof(address.s_addr));
}

void gvcp_force_ip(
  const std::string & interface_name, const std::string & source_address,
  const std::string & broadcast_address, const std::string & mac_address,
  const std::string & address, const std::string & subnet_mask, const std::string & gateway)
{
  constexpr std::uint16_t gvcp_port = 3956;
  constexpr std::uint16_t force_ip_command = 0x0004;
  constexpr std::uint16_t payload_size = 56;
  static std::atomic<std::uint16_t> request_sequence{1};

  std::array<std::uint8_t, 8 + payload_size> packet{};
  packet[0] = 0x42;
  packet[1] = 0x11;  // Acknowledge requested and broadcast acknowledge allowed.
  write_u16(packet.data() + 2, force_ip_command);
  write_u16(packet.data() + 4, payload_size);
  const auto request_id = request_sequence.fetch_add(1);
  write_u16(packet.data() + 6, request_id == 0 ? 1 : request_id);
  const auto mac = mac_bytes(mac_address);
  std::copy(mac.begin(), mac.end(), packet.begin() + 8 + 2);
  write_ipv4(packet.data() + 8 + 20, address);
  write_ipv4(packet.data() + 8 + 36, subnet_mask);
  write_ipv4(packet.data() + 8 + 52, gateway);

  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {throw std::runtime_error("cannot create GVCP socket");}
  const auto close_socket = [&socket_fd]() {close(socket_fd);};
  int enabled = 1;
  if (setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled)) != 0) {
    const auto message = std::string("cannot enable GVCP broadcast on ") + interface_name +
      ": " + std::strerror(errno);
    close_socket();
    throw std::runtime_error(message);
  }
  if (setsockopt(
      socket_fd, SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(),
      interface_name.size() + 1) != 0)
  {
    const auto message = std::string("cannot pin GVCP ForceIP to ") + interface_name +
      ": " + std::strerror(errno);
    close_socket();
    throw std::runtime_error(message);
  }
  sockaddr_in source{};
  source.sin_family = AF_INET;
  source.sin_port = 0;
  if (inet_pton(AF_INET, source_address.c_str(), &source.sin_addr) != 1 ||
    bind(socket_fd, reinterpret_cast<const sockaddr *>(&source), sizeof(source)) != 0)
  {
    const auto message = std::string("cannot bind GVCP ForceIP source ") + source_address +
      " on " + interface_name + ": " + std::strerror(errno);
    close_socket();
    throw std::runtime_error(message);
  }
  sockaddr_in directed_destination{};
  directed_destination.sin_family = AF_INET;
  directed_destination.sin_port = htons(gvcp_port);
  if (inet_pton(
      AF_INET, broadcast_address.c_str(), &directed_destination.sin_addr) != 1)
  {
    close_socket();
    throw std::runtime_error("invalid GVCP broadcast address " + broadcast_address);
  }
  auto limited_destination = directed_destination;
  limited_destination.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  for (int attempt = 0; attempt < 3; ++attempt) {
    for (const auto * destination : {&directed_destination, &limited_destination}) {
      const auto sent = sendto(
        socket_fd, packet.data(), packet.size(), 0,
        reinterpret_cast<const sockaddr *>(destination), sizeof(*destination));
      if (sent != static_cast<ssize_t>(packet.size())) {
        const auto message = std::string("GVCP ForceIP send failed: ") + std::strerror(errno);
        close_socket();
        throw std::runtime_error(message);
      }
    }
    std::this_thread::sleep_for(100ms);
  }
  close_socket();
}

std::string buffer_status_name(ArvBufferStatus status)
{
  switch (status) {
    case ARV_BUFFER_STATUS_SUCCESS: return "success";
    case ARV_BUFFER_STATUS_CLEARED: return "cleared";
    case ARV_BUFFER_STATUS_TIMEOUT: return "timeout";
    case ARV_BUFFER_STATUS_MISSING_PACKETS: return "missing packets";
    case ARV_BUFFER_STATUS_WRONG_PACKET_ID: return "wrong packet id";
    case ARV_BUFFER_STATUS_SIZE_MISMATCH: return "size mismatch";
    case ARV_BUFFER_STATUS_FILLING: return "still filling";
    case ARV_BUFFER_STATUS_ABORTED: return "aborted";
    case ARV_BUFFER_STATUS_PAYLOAD_NOT_SUPPORTED: return "payload not supported";
    default: return "unknown";
  }
}

ArvAuto auto_mode(const std::string & value)
{
  const auto mode = lower(value);
  if (mode == "once") {return ARV_AUTO_ONCE;}
  if (mode == "continuous") {return ARV_AUTO_CONTINUOUS;}
  return ARV_AUTO_OFF;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream stream;
  for (const auto character : value) {
    switch (character) {
      case '\\': stream << "\\\\"; break;
      case '"': stream << "\\\""; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default: stream << character; break;
    }
  }
  return stream.str();
}

template<typename T>
T setting(const YAML::Node & node, const char * name, T fallback)
{
  return node && node[name] ? node[name].as<T>() : fallback;
}

cv::Mat to_bgr(ArvBuffer * buffer)
{
  const int width = arv_buffer_get_image_width(buffer);
  const int height = arv_buffer_get_image_height(buffer);
  size_t size = 0;
  const auto * data = static_cast<const std::uint8_t *>(arv_buffer_get_image_data(buffer, &size));
  const auto format = arv_buffer_get_image_pixel_format(buffer);
  if (data == nullptr || width <= 0 || height <= 0) {
    throw std::runtime_error("Aravis returned an empty image");
  }
  cv::Mat result;
  if (format == ARV_PIXEL_FORMAT_BGR_8_PACKED) {
    const auto needed = static_cast<std::size_t>(width) * height * 3;
    if (size < needed) {throw std::runtime_error("short BGR8 image payload");}
    result = cv::Mat(height, width, CV_8UC3, const_cast<std::uint8_t *>(data)).clone();
  } else if (format == ARV_PIXEL_FORMAT_RGB_8_PACKED) {
    const auto needed = static_cast<std::size_t>(width) * height * 3;
    if (size < needed) {throw std::runtime_error("short RGB8 image payload");}
    cv::cvtColor(
      cv::Mat(height, width, CV_8UC3, const_cast<std::uint8_t *>(data)), result,
      cv::COLOR_RGB2BGR);
  } else {
    const auto needed = static_cast<std::size_t>(width) * height;
    if (size < needed) {throw std::runtime_error("short 8-bit image payload");}
    const cv::Mat source(height, width, CV_8UC1, const_cast<std::uint8_t *>(data));
    int conversion = cv::COLOR_GRAY2BGR;
    if (format == ARV_PIXEL_FORMAT_BAYER_RG_8) {conversion = cv::COLOR_BayerRG2BGR;}
    else if (format == ARV_PIXEL_FORMAT_BAYER_BG_8) {conversion = cv::COLOR_BayerBG2BGR;}
    else if (format == ARV_PIXEL_FORMAT_BAYER_GR_8) {conversion = cv::COLOR_BayerGR2BGR;}
    else if (format == ARV_PIXEL_FORMAT_BAYER_GB_8) {conversion = cv::COLOR_BayerGB2BGR;}
    else if (format != ARV_PIXEL_FORMAT_MONO_8) {
      throw std::runtime_error("unsupported GenICam pixel format " + std::to_string(format));
    }
    cv::cvtColor(source, result, conversion);
  }
  return result;
}

class CameraEndpoint
{
public:
  CameraEndpoint(rclcpp::Node * node, const Assignment & assignment, const GenicamConfig & config)
  : assignment_sensor_id_(assignment.sensor_id), frame_id_(assignment.frame_id),
    config_(config), logger_(node->get_logger())
  {
    const std::string base = "/vixel/sensors/" + assignment.sensor_id;
    image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>(
      base + "/image_raw", rclcpp::SensorDataQoS());
    compressed_publisher_ = node->create_publisher<sensor_msgs::msg::CompressedImage>(
      base + "/image_raw/compressed", rclcpp::SensorDataQoS());
    const auto capture_qos = rclcpp::QoS(rclcpp::KeepLast(128)).reliable();
    capture_publisher_ = node->create_publisher<vixel_interfaces::msg::CaptureFrameChunk>(
      base + "/image_capture/chunks", capture_qos);
    info_publisher_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
      base + "/camera_info", rclcpp::SensorDataQoS());
    info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      node, assignment.sensor_id, assignment.calibration_url, base);
  }

  void publish(
    const cv::Mat & image, const builtin_interfaces::msg::Time & stamp,
    const std::string & capture_id = {}, bool publish_capture_chunks = false)
  {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;
    const bool publish_raw = image_publisher_->get_subscription_count() != 0;
    if (publish_raw) {
      auto message = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
      image_publisher_->publish(*message);
    }
    if (!capture_id.empty() || publish_raw || info_publisher_->get_subscription_count() != 0) {
      auto info = info_manager_->getCameraInfo();
      info.header = header;
      if (info.width == 0 || info.height == 0) {
        info.width = image.cols;
        info.height = image.rows;
      }
      info_publisher_->publish(info);
    }
    if (publish_capture_chunks) {
      std::vector<std::uint8_t> encoded;
      if (!cv::imencode(
          ".png", image, encoded,
          {cv::IMWRITE_PNG_COMPRESSION, config_.png_compression}))
      {
        throw std::runtime_error("OpenCV failed to encode full-resolution capture as PNG");
      }
      constexpr std::size_t chunk_size = 256U * 1024U;
      const auto chunk_count = static_cast<std::uint32_t>(
        (encoded.size() + chunk_size - 1U) / chunk_size);
      for (std::uint32_t index = 0; index < chunk_count; ++index) {
        const auto begin = encoded.begin() + static_cast<std::ptrdiff_t>(index * chunk_size);
        const auto remaining = encoded.size() - index * chunk_size;
        const auto length = std::min(chunk_size, remaining);
        vixel_interfaces::msg::CaptureFrameChunk chunk;
        chunk.header = header;
        chunk.capture_id = capture_id;
        chunk.sensor_id = assignment_sensor_id_;
        chunk.format = "png";
        chunk.width = image.cols;
        chunk.height = image.rows;
        chunk.chunk_index = index;
        chunk.chunk_count = chunk_count;
        chunk.data.assign(begin, begin + static_cast<std::ptrdiff_t>(length));
        capture_publisher_->publish(chunk);
      }
      RCLCPP_INFO(
        logger_, "Published capture %s for %s as %u PNG chunk(s)",
        capture_id.c_str(), assignment_sensor_id_.c_str(), chunk_count);
    }

    cv::Mat preview = image;
    if (image.cols > config_.preview_width) {
      const double scale = static_cast<double>(config_.preview_width) / image.cols;
      cv::resize(image, preview, cv::Size(), scale, scale, cv::INTER_AREA);
    }
    sensor_msgs::msg::CompressedImage compressed;
    compressed.header = header;
    std::string extension;
    std::vector<int> options;
    if (config_.preview_format == "png") {
      compressed.format = "png";
      extension = ".png";
      options = {cv::IMWRITE_PNG_COMPRESSION, config_.png_compression};
    } else {
      compressed.format = "jpeg";
      extension = ".jpg";
      options = {cv::IMWRITE_JPEG_QUALITY, config_.jpeg_quality};
    }
    if (!cv::imencode(extension, preview, compressed.data, options)) {
      throw std::runtime_error(
              "OpenCV failed to encode preview as " + config_.preview_format);
    }
    compressed_publisher_->publish(compressed);
  }

private:
  std::string assignment_sensor_id_;
  std::string frame_id_;
  GenicamConfig config_;
  rclcpp::Logger logger_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
  rclcpp::Publisher<vixel_interfaces::msg::CaptureFrameChunk>::SharedPtr capture_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
};

class CameraSession
{
public:
  struct FrameCompletion
  {
    std::mutex mutex;
    std::condition_variable changed;
    bool armed{false};
    bool done{false};
    bool success{false};
    std::uint64_t trigger_started_ns{0};
    std::uint64_t device_timestamp_ns{0};
    std::int64_t ptp_offset_ns{0};
    bool synchronized{false};
    std::string error;
  };

  struct FrameRequest
  {
    builtin_interfaces::msg::Time stamp;
    std::string capture_id;
    bool publish_capture_chunks{false};
    std::chrono::steady_clock::time_point release_at{};
    bool force_software_trigger{false};
    std::shared_ptr<FrameCompletion> completion;
  };

  struct PtpState
  {
    std::string status;
    bool offset_exposed{false};
    std::int64_t offset_ns{0};
  };

  CameraSession(
    rclcpp::Node * node, Assignment assignment, DeviceRecord record, NetworkConfig network,
    GenicamConfig config)
  : node_(node), assignment_(std::move(assignment)), record_(std::move(record)),
    network_(std::move(network)), config_(std::move(config)),
    endpoint_(std::make_unique<CameraEndpoint>(node_, assignment_, config_))
  {
  }

  ~CameraSession() {shutdown();}
  CameraSession(const CameraSession &) = delete;
  CameraSession & operator=(const CameraSession &) = delete;

  void initialize()
  {
    GError * error = nullptr;
    camera_ = arv_camera_new(record_.device_id.c_str(), &error);
    throw_on_error(error, "opening GenICam camera " + record_.serial);
    if (camera_ == nullptr) {throw std::runtime_error("Aravis returned no camera");}
    apply_configuration();

    error = nullptr;
    stream_ = arv_camera_create_stream(camera_, nullptr, nullptr, &error);
    throw_on_error(error, "creating GenICam stream");
    if (stream_ == nullptr) {throw std::runtime_error("Aravis returned no stream");}
    if (ARV_IS_GV_STREAM(stream_)) {
      g_object_set(
        stream_, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_ALWAYS,
        "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_FIXED,
        "socket-buffer-size", config_.socket_buffer_bytes, nullptr);
    }
    error = nullptr;
    const auto payload = arv_camera_get_payload(camera_, &error);
    throw_on_error(error, "reading image payload size");
    for (int index = 0; index < config_.buffer_count; ++index) {
      arv_stream_push_buffer(stream_, arv_buffer_new_allocate(payload));
    }
    error = nullptr;
    arv_camera_start_acquisition(camera_, &error);
    throw_on_error(error, "starting acquisition");
    if (transfer_start_required_) {
      error = nullptr;
      arv_camera_execute_command(camera_, "TransferStart", &error);
      throw_on_error(error, "starting user-controlled image transfer");
    }
    streaming_.store(true);
    ready_.store(true);
    worker_ = std::thread(&CameraSession::worker_loop, this);
  }

  void shutdown()
  {
    if (stopping_.exchange(true)) {return;}
    {
      std::lock_guard<std::mutex> request_lock(request_mutex_);
      if (pending_request_ && pending_request_->completion) {
        auto completion = pending_request_->completion;
        {
          std::lock_guard<std::mutex> completion_lock(completion->mutex);
          completion->error = "camera session stopped";
          completion->done = true;
        }
        completion->changed.notify_all();
      }
      pending_request_.reset();
      request_active_ = false;
    }
    request_cv_.notify_all();
    if (worker_.joinable()) {worker_.join();}
    if (camera_ != nullptr && streaming_.exchange(false)) {
      GError * error = nullptr;
      if (transfer_start_required_) {
        arv_camera_execute_command(camera_, "TransferStop", &error);
        if (error != nullptr) {g_error_free(error); error = nullptr;}
      }
      arv_camera_stop_acquisition(camera_, &error);
      if (error != nullptr) {g_error_free(error);}
    }
    ready_.store(false);
    if (stream_ != nullptr) {g_object_unref(stream_); stream_ = nullptr;}
    if (camera_ != nullptr) {g_object_unref(camera_); camera_ = nullptr;}
  }

  bool preview_due(const std::chrono::steady_clock::time_point tick)
  {
    if (assignment_.operating_mode != "preview" || tick < next_preview_) {return false;}
    const double rate = std::clamp(assignment_.preview_rate_hz, 0.1, 10.0);
    next_preview_ = tick + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rate));
    return true;
  }

  std::shared_ptr<FrameCompletion> request_frame(
    const builtin_interfaces::msg::Time & stamp, const std::string & capture_id = {},
    bool publish_capture_chunks = false,
    std::chrono::steady_clock::time_point release_at = {},
    bool force_software_trigger = false)
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    if (!ready_.load() || request_active_) {return {};}
    std::shared_ptr<FrameCompletion> completion;
    if (!capture_id.empty()) {
      completion = std::make_shared<FrameCompletion>();
    }
    pending_request_ = FrameRequest{
      stamp, capture_id, publish_capture_chunks, release_at, force_software_trigger, completion};
    request_active_ = true;
    request_cv_.notify_one();
    return completion;
  }

  bool ready() const {return ready_.load();}
  bool software_trigger() const {return software_trigger_;}
  bool ptp_action() const {return ptp_action_;}
  const NetworkConfig & network() const {return network_;}
  std::int64_t ptp_offset_ns()
  {
    if (!ptp_action_) {return 0;}
    const auto state = read_ptp_state();
    return state.offset_exposed ? state.offset_ns : 0;
  }
  bool ptp_locked()
  {
    if (!ptp_action_) {return false;}
    const auto state = read_ptp_state();
    if (lower(state.status) != "slave") {return false;}
    return !state.offset_exposed ||
           std::llabs(state.offset_ns) <= config_.ptp_tolerance_ns;
  }
  std::uint64_t ptp_time_ns()
  {
    if (!ptp_action_) {throw std::runtime_error("camera has no PTP action clock");}
    std::lock_guard<std::mutex> lock(camera_control_mutex_);
    GError * error = nullptr;
    arv_camera_execute_command(camera_, ptp_latch_feature_.c_str(), &error);
    throw_on_error(error, "latching PTP time");
    const auto value = arv_camera_get_integer(camera_, ptp_latch_value_feature_.c_str(), &error);
    throw_on_error(error, "reading latched PTP time");
    return static_cast<std::uint64_t>(value);
  }
  const Assignment & assignment() const {return assignment_;}

  Sensor status()
  {
    Sensor result;
    result.stamp = node_->now();
    result.sensor_id = assignment_.sensor_id;
    result.provider = "genicam";
    result.kind = "camera";
    result.vendor = record_.vendor;
    result.model = record_.model;
    result.serial = record_.serial;
    result.mac_address = normalized_mac(record_.physical_id);
    result.enrolled = true;
    result.enabled = assignment_.enabled;
    result.online = ready_.load();
    result.lifecycle_state = ready_.load() ? assignment_.operating_mode : "error";
    result.calibration_url = assignment_.calibration_url;
    result.topic_base = "/vixel/sensors/" + assignment_.sensor_id;
    result.network_id = assignment_.network_id;
    result.assigned_address = assignment_.assigned_address;
    result.current_address = record_.address;
    result.interface_name = record_.interface_name;
    result.sync_group = assignment_.sync_group;
    result.operating_mode = assignment_.operating_mode;
    result.capabilities = {
      "image", "compressed_preview", "genicam", "camera_settings",
      "software_capture"};
    result.capabilities.push_back(ptp_action_ ? "ptp_scheduled_action" : "unsynchronized_group_capture");
    result.applied_settings_json = applied_settings_;
    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      result.last_error = last_error_;
    }
    if (ptp_action_) {
      const auto state = read_ptp_state();
      result.status_detail = "PTP " + state.status + ", offset " +
        (!state.offset_exposed ? std::string("not exposed by camera") :
        state.offset_ns == std::numeric_limits<std::int64_t>::max() ?
        std::string("unreadable") : std::to_string(state.offset_ns) + " ns");
    } else {
      result.status_detail = ptp_capability_detail_;
    }
    if (!device_version_.empty()) {
      result.status_detail += "; firmware " + device_version_;
    }
    return result;
  }

  std::vector<vixel_interfaces::msg::CameraFeature> features()
  {
    std::vector<vixel_interfaces::msg::CameraFeature> result;
    add_float_feature(result, "ExposureTime", "us");
    add_float_feature(result, "Gain", "dB");
    add_float_feature(result, "AcquisitionFrameRate", "Hz");
    add_enum_feature(result, "PixelFormat");
    add_enum_feature(result, "ExposureAuto");
    add_enum_feature(result, "GainAuto");
    return result;
  }

private:
  PtpState read_ptp_state()
  {
    std::lock_guard<std::mutex> lock(camera_control_mutex_);
    PtpState result;
    GError * error = nullptr;
    result.status = value_or_empty(
      arv_camera_get_string(camera_, ptp_status_feature_.c_str(), &error));
    if (error != nullptr) {
      g_error_free(error);
      result.status = "unreadable";
      return result;
    }

    // Some cameras, including LUCID firmware RQQ51, hide the optional offset
    // node until PTP has reached Slave. The initial configuration probe can
    // therefore miss a node that becomes readable later. Re-probe it while
    // locked so hotplug recovery does not require rebuilding the session.
    const auto now = std::chrono::steady_clock::now();
    if (lower(result.status) == "slave" && ptp_offset_feature_.empty() &&
      now >= next_ptp_offset_probe_)
    {
      next_ptp_offset_probe_ = now + 1s;
      ptp_offset_feature_ = first_available_feature(
        {"PtpOffsetFromMaster", "GevIEEE1588OffsetFromMaster"});
      if (!ptp_offset_feature_.empty()) {
        RCLCPP_INFO(
          node_->get_logger(), "%s PTP offset feature %s became available after lock",
          assignment_.sensor_id.c_str(), ptp_offset_feature_.c_str());
      }
    }

    result.offset_exposed = !ptp_offset_feature_.empty();
    if (!result.offset_exposed) {return result;}
    error = nullptr;
    result.offset_ns = arv_camera_get_integer(camera_, ptp_offset_feature_.c_str(), &error);
    if (error != nullptr) {
      g_error_free(error);
      result.offset_ns = std::numeric_limits<std::int64_t>::max();
    }
    return result;
  }

  std::string first_available_feature(
    std::initializer_list<const char *> candidates)
  {
    for (const auto * candidate : candidates) {
      GError * error = nullptr;
      const bool available = arv_camera_is_feature_available(camera_, candidate, &error);
      if (error != nullptr) {g_error_free(error);}
      if (available) {return candidate;}
    }
    return {};
  }

  void apply_configuration()
  {
    YAML::Node settings;
    if (!assignment_.provider_settings_json.empty()) {
      try {settings = YAML::Load(assignment_.provider_settings_json);} catch (const YAML::Exception & error) {
        throw std::runtime_error(std::string("invalid provider settings: ") + error.what());
      }
    }
    GError * error = nullptr;
    if (config_.imaging.use_max_width || config_.imaging.use_max_height) {
      int min_width = 0, max_width = 0, min_height = 0, max_height = 0;
      arv_camera_get_width_bounds(camera_, &min_width, &max_width, &error);
      throw_on_error(error, "reading width bounds");
      error = nullptr;
      arv_camera_get_height_bounds(camera_, &min_height, &max_height, &error);
      throw_on_error(error, "reading height bounds");
      const int width = setting(settings, "width", config_.imaging.use_max_width ? max_width : min_width);
      const int height = setting(settings, "height", config_.imaging.use_max_height ? max_height : min_height);
      error = nullptr;
      arv_camera_set_region(camera_, 0, 0, width, height, &error);
      throw_on_error(error, "setting image region");
    }

    const auto pixel_format = setting(settings, "pixel_format", config_.imaging.pixel_format);
    error = nullptr;
    arv_camera_set_pixel_format_from_string(camera_, pixel_format.c_str(), &error);
    if (error != nullptr) {
      RCLCPP_WARN(
        node_->get_logger(), "Camera %s rejected pixel format %s; keeping camera default: %s",
        assignment_.sensor_id.c_str(), pixel_format.c_str(), error->message);
      g_error_free(error);
    }

    const auto exposure_auto = setting(settings, "exposure_auto", config_.imaging.exposure_auto);
    error = nullptr;
    if (arv_camera_is_exposure_auto_available(camera_, &error) && error == nullptr) {
      arv_camera_set_exposure_time_auto(camera_, auto_mode(exposure_auto), &error);
      throw_on_error(error, "setting automatic exposure");
    } else if (error != nullptr) {g_error_free(error); error = nullptr;}
    const auto exposure = setting(settings, "exposure_us", config_.imaging.exposure_time_us);
    if (auto_mode(exposure_auto) == ARV_AUTO_OFF) {
      arv_camera_set_exposure_time(camera_, exposure, &error);
      throw_on_error(error, "setting exposure time");
    }

    const auto gain_auto = setting(settings, "gain_auto", config_.imaging.gain_auto);
    error = nullptr;
    if (arv_camera_is_gain_auto_available(camera_, &error) && error == nullptr) {
      arv_camera_set_gain_auto(camera_, auto_mode(gain_auto), &error);
      throw_on_error(error, "setting automatic gain");
    } else if (error != nullptr) {g_error_free(error); error = nullptr;}
    const auto gain = setting(settings, "gain_db", config_.imaging.gain_db);
    if (auto_mode(gain_auto) == ARV_AUTO_OFF) {
      arv_camera_set_gain(camera_, gain, &error);
      throw_on_error(error, "setting gain");
    }

    double frame_rate = setting(settings, "frame_rate_hz", config_.imaging.frame_rate_hz);
    if (assignment_.operating_mode == "preview" && !settings["frame_rate_hz"]) {
      frame_rate = std::clamp(assignment_.preview_rate_hz, 0.1, 10.0);
    }
    error = nullptr;
    if (arv_camera_is_frame_rate_available(camera_, &error) && error == nullptr) {
      arv_camera_set_frame_rate(camera_, frame_rate, &error);
      throw_on_error(error, "setting frame rate");
    } else if (error != nullptr) {g_error_free(error); error = nullptr;}

    if (arv_camera_is_gv_device(camera_)) {
      const auto packet_size = setting(
        settings, "packet_size", static_cast<int>(network_.packet_size));
      const auto packet_delay = setting(
        settings, "packet_delay_ns", static_cast<std::int64_t>(network_.packet_delay));
      arv_camera_gv_set_packet_size(camera_, packet_size, &error);
      throw_on_error(error, "setting GigE packet size");
      arv_camera_gv_set_packet_delay(camera_, packet_delay, &error);
      throw_on_error(error, "setting GigE packet delay");
    }

    error = nullptr;
    if (arv_camera_is_feature_available(camera_, "DeviceVersion", &error) && error == nullptr) {
      device_version_ = value_or_empty(arv_camera_get_string(camera_, "DeviceVersion", &error));
      if (error != nullptr) {g_error_free(error); error = nullptr; device_version_.clear();}
    } else if (error != nullptr) {g_error_free(error); error = nullptr;}

    const auto trigger_source = setting(settings, "trigger_source", std::string("FreeRun"));
    const auto normalized_trigger = lower(trigger_source);
    error = nullptr;
    arv_camera_set_acquisition_mode(camera_, ARV_ACQUISITION_MODE_CONTINUOUS, &error);
    throw_on_error(error, "setting continuous acquisition mode");

    if (normalized_trigger == "software") {
      arv_camera_set_trigger(camera_, "Software", &error);
      throw_on_error(error, "enabling software trigger");
      software_trigger_ = true;
    } else if (normalized_trigger == "action0") {
      bool supported = arv_camera_is_gv_device(camera_);
      ptp_enable_feature_ = first_available_feature({"PtpEnable", "GevIEEE1588"});
      ptp_status_feature_ = first_available_feature({"PtpStatus", "GevIEEE1588Status"});
      ptp_offset_feature_ = first_available_feature(
        {"PtpOffsetFromMaster", "GevIEEE1588OffsetFromMaster"});
      ptp_latch_feature_ = first_available_feature(
        {"PtpDataSetLatch", "GevTimestampControlLatch"});
      ptp_latch_value_feature_ = first_available_feature(
        {"PtpDataSetLatchValue", "PtpTimestampLatchValue", "GevTimestampValue"});
      std::vector<std::string> missing;
      const std::array<std::pair<const char *, const std::string *>, 4> ptp_features{{
        {"PTP enable", &ptp_enable_feature_},
        {"PTP status", &ptp_status_feature_},
        {"PTP clock latch", &ptp_latch_feature_},
        {"PTP latched value", &ptp_latch_value_feature_},
      }};
      for (const auto & feature : ptp_features) {
        if (feature.second->empty()) {missing.emplace_back(feature.first);}
      }
      for (const auto * feature : {"ActionDeviceKey", "ActionGroupKey", "ActionGroupMask"}) {
        if (first_available_feature({feature}).empty()) {missing.emplace_back(feature);}
      }
      supported = supported && missing.empty();
      const auto trigger_selector = first_available_feature({"TriggerSelector"});
      if (!trigger_selector.empty() && feature_writable(camera_, trigger_selector.c_str())) {
        error = nullptr;
        arv_camera_set_string(camera_, trigger_selector.c_str(), "FrameStart", &error);
        if (error != nullptr) {
          RCLCPP_WARN(
            node_->get_logger(), "%s could not select TriggerSelector=FrameStart: %s",
            assignment_.sensor_id.c_str(), error->message);
          g_error_free(error);
          error = nullptr;
        }
      }
      guint trigger_count = 0;
      const auto trigger_values = arv_camera_dup_available_enumerations_as_strings(
        camera_, "TriggerSource", &trigger_count, &error);
      bool has_action0 = false;
      if (error == nullptr) {
        for (guint index = 0; trigger_values != nullptr && index < trigger_count; ++index) {
          has_action0 = has_action0 || lower(value_or_empty(trigger_values[index])) == "action0";
        }
      } else {
        g_error_free(error);
        error = nullptr;
      }
      g_free(trigger_values);
      supported = supported && has_action0;
      if (!has_action0) {missing.emplace_back("TriggerSource=Action0");}
      if (supported) {
        arv_camera_set_boolean(camera_, ptp_enable_feature_.c_str(), true, &error);
        throw_on_error(error, "enabling PTP");
        const auto slave_only = first_available_feature(
          {"PtpSlaveOnly", "GevIEEE1588SlaveOnly"});
        if (!slave_only.empty() && feature_writable(camera_, slave_only.c_str())) {
          arv_camera_set_boolean(camera_, slave_only.c_str(), true, &error);
          throw_on_error(error, "configuring camera as a PTP slave");
        }
        error = nullptr;
        if (arv_camera_is_feature_available(camera_, "ActionUnconditionalMode", &error) &&
          error == nullptr)
        {
          arv_camera_set_string(camera_, "ActionUnconditionalMode", "On", &error);
          throw_on_error(error, "enabling unconditional action command");
        } else if (error != nullptr) {g_error_free(error); error = nullptr;}
        error = nullptr;
        if (arv_camera_is_feature_available(camera_, "ActionSelector", &error) &&
          error == nullptr)
        {
          arv_camera_set_integer(camera_, "ActionSelector", 0, &error);
          throw_on_error(error, "selecting Action0");
        } else if (error != nullptr) {g_error_free(error); error = nullptr;}
        arv_camera_set_integer(
          camera_, "ActionDeviceKey", config_.action_device_key, &error);
        throw_on_error(error, "setting ActionDeviceKey");
        arv_camera_set_integer(
          camera_, "ActionGroupKey", action_group_key(assignment_.sync_group), &error);
        throw_on_error(error, "setting ActionGroupKey");
        arv_camera_set_integer(camera_, "ActionGroupMask", action_group_mask, &error);
        throw_on_error(error, "setting ActionGroupMask");
        arv_camera_set_trigger(camera_, "Action0", &error);
        throw_on_error(error, "enabling Action0 trigger");
        ptp_action_ = true;
        action_trigger_armed_ = true;
        if (assignment_.operating_mode == "preview") {
          error = nullptr;
          arv_camera_set_trigger(camera_, "Software", &error);
          throw_on_error(error, "selecting software trigger for preview mode");
          software_trigger_ = true;
          action_trigger_armed_ = false;
        }
        RCLCPP_INFO(
          node_->get_logger(),
          "%s PTP action configuration: trigger=%s device_key=%lld "
          "group_key=%lld group_mask=0x%llx lead=%d ms",
          assignment_.sensor_id.c_str(),
          assignment_.operating_mode == "preview" ? "Software" : "Action0",
          static_cast<long long>(config_.action_device_key),
          static_cast<long long>(action_group_key(assignment_.sync_group)),
          static_cast<unsigned long long>(action_group_mask),
          config_.ptp_action_lead_time_ms);
        ptp_capability_detail_ = "PTP supported; waiting for lock";
      } else {
        std::ostringstream detail;
        detail << "PTP scheduled action unsupported; missing ";
        for (std::size_t index = 0; index < missing.size(); ++index) {
          if (index != 0) {detail << ", ";}
          detail << missing[index];
        }
        ptp_capability_detail_ = detail.str();
        RCLCPP_WARN(
          node_->get_logger(),
          "%s: %s%s%s; using unsynchronized software trigger",
          assignment_.sensor_id.c_str(), ptp_capability_detail_.c_str(),
          device_version_.empty() ? "" : "; firmware ", device_version_.c_str());
        arv_camera_set_trigger(camera_, "Software", &error);
        throw_on_error(error, "enabling software fallback trigger");
        software_trigger_ = true;
      }
    } else if (normalized_trigger == "freerun" || normalized_trigger == "free_run") {
      error = nullptr;
      const bool trigger_mode_available =
        arv_camera_is_feature_available(camera_, "TriggerMode", &error);
      if (error != nullptr) {
        g_error_free(error);
        error = nullptr;
      } else if (trigger_mode_available && feature_writable(camera_, "TriggerMode")) {
        arv_camera_clear_triggers(camera_, &error);
        if (error != nullptr) {
          const auto clear_error = error_text(error, "disabling camera triggers");
          error = nullptr;
          const auto current_mode = value_or_empty(
            arv_camera_get_string(camera_, "TriggerMode", &error));
          throw_on_error(error, "checking TriggerMode after disable failed");
          if (lower(current_mode) != "off") {throw std::runtime_error(clear_error);}
          RCLCPP_WARN(
            node_->get_logger(),
            "%s rejected a redundant TriggerMode write but reports TriggerMode=Off; "
            "continuing in free-run",
            assignment_.sensor_id.c_str());
        }
      } else if (trigger_mode_available) {
        const auto current_mode = value_or_empty(
          arv_camera_get_string(camera_, "TriggerMode", &error));
        throw_on_error(error, "reading read-only TriggerMode");
        if (lower(current_mode) != "off") {
          throw std::runtime_error(
                  "TriggerMode is read-only and currently " + current_mode +
                  "; the camera cannot enter FreeRun mode");
        }
        RCLCPP_INFO(
          node_->get_logger(), "%s has read-only TriggerMode=Off; keeping camera in free-run",
          assignment_.sensor_id.c_str());
      }
    } else {
      throw std::runtime_error(
              "unsupported trigger_source " + trigger_source +
              "; use FreeRun, Software, or Action0");
    }

    const auto feature_values = settings["features"];
    if (feature_values && feature_values.IsMap()) {
      for (const auto & item : feature_values) {
        const auto name = item.first.as<std::string>();
        const auto specification = item.second;
        const auto type = specification["type"].as<std::string>();
        error = nullptr;
        if (type == "boolean") {
          arv_camera_set_boolean(camera_, name.c_str(), specification["value"].as<bool>(), &error);
        } else if (type == "integer") {
          arv_camera_set_integer(camera_, name.c_str(), specification["value"].as<std::int64_t>(), &error);
        } else if (type == "float") {
          arv_camera_set_float(camera_, name.c_str(), specification["value"].as<double>(), &error);
        } else if (type == "string" || type == "enum") {
          const auto value = specification["value"].as<std::string>();
          arv_camera_set_string(camera_, name.c_str(), value.c_str(), &error);
        } else if (type == "command") {
          arv_camera_execute_command(camera_, name.c_str(), &error);
        } else {
          throw std::runtime_error("unknown GenICam feature type for " + name);
        }
        throw_on_error(error, "setting GenICam feature " + name);
      }
    }
    configure_transfer_control(settings);
    applied_settings_ = assignment_.provider_settings_json.empty() ? "{}" :
      assignment_.provider_settings_json;
  }

  void configure_transfer_control(const YAML::Node & settings)
  {
    GError * error = nullptr;
    if (!arv_camera_is_feature_available(camera_, "TransferControlMode", &error)) {
      if (error != nullptr) {g_error_free(error);}
      return;
    }
    throw_on_error(error, "checking TransferControlMode");

    guint count = 0;
    const auto values = arv_camera_dup_available_enumerations_as_strings(
      camera_, "TransferControlMode", &count, &error);
    throw_on_error(error, "reading TransferControlMode values");
    std::vector<std::string> available;
    for (guint index = 0; values != nullptr && index < count; ++index) {
      available.push_back(value_or_empty(values[index]));
    }
    g_free(values);

    auto selected = setting(settings, "transfer_control_mode", std::string{});
    if (selected.empty()) {
      for (const auto & preferred : {std::string("Basic"), std::string("Automatic")}) {
        const auto found = std::find_if(
          available.begin(), available.end(), [&preferred](const auto & candidate) {
            return lower(candidate) == lower(preferred);
          });
        if (found != available.end()) {selected = *found; break;}
      }
      if (selected.empty()) {
        error = nullptr;
        selected = value_or_empty(arv_camera_get_string(
            camera_, "TransferControlMode", &error));
        throw_on_error(error, "reading TransferControlMode");
      }
    } else {
      const auto found = std::find_if(
        available.begin(), available.end(), [&selected](const auto & candidate) {
          return lower(candidate) == lower(selected);
        });
      if (found == available.end()) {
        throw std::runtime_error("unsupported TransferControlMode " + selected);
      }
      selected = *found;
    }

    error = nullptr;
    arv_camera_set_string(camera_, "TransferControlMode", selected.c_str(), &error);
    throw_on_error(error, "setting TransferControlMode");
    if (lower(selected) != "usercontrolled") {return;}

    error = nullptr;
    if (arv_camera_is_feature_available(camera_, "TransferOperationMode", &error) &&
      error == nullptr)
    {
      arv_camera_set_string(camera_, "TransferOperationMode", "Continuous", &error);
      throw_on_error(error, "setting TransferOperationMode");
    } else if (error != nullptr) {
      g_error_free(error);
      error = nullptr;
    }
    if (!arv_camera_is_feature_available(camera_, "TransferStart", &error) || error != nullptr) {
      const auto message = error == nullptr ? std::string("TransferStart is unavailable") :
        error_text(error, "checking TransferStart");
      throw std::runtime_error(
              "UserControlled transfer requested but " + message);
    }
    transfer_start_required_ = true;
  }

  void worker_loop()
  {
    while (!stopping_.load()) {
      std::optional<FrameRequest> request;
      {
        std::unique_lock<std::mutex> lock(request_mutex_);
        request_cv_.wait_for(lock, 200ms, [this]() {
          return stopping_.load() || pending_request_.has_value();
        });
        if (stopping_.load()) {break;}
        request.swap(pending_request_);
      }
      if (!request) {continue;}
      if (request->completion) {
        {
          std::lock_guard<std::mutex> completion_lock(request->completion->mutex);
          request->completion->armed = true;
        }
        request->completion->changed.notify_all();
      }
      bool restore_action_trigger = false;
      try {
        if (request->release_at != std::chrono::steady_clock::time_point{}) {
          std::this_thread::sleep_until(request->release_at);
        }
        const bool software_for_request = software_trigger_ || request->force_software_trigger;
        if (request->force_software_trigger && action_trigger_armed_) {
          GError * error = nullptr;
          std::lock_guard<std::mutex> lock(camera_control_mutex_);
          arv_camera_set_trigger(camera_, "Software", &error);
          throw_on_error(error, "temporarily selecting software trigger while PTP is unlocked");
          restore_action_trigger = true;
        }
        if (software_for_request) {
          if (request->completion) {
            request->completion->trigger_started_ns =
              static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
          }
          GError * error = nullptr;
          {
            std::lock_guard<std::mutex> lock(camera_control_mutex_);
            arv_camera_software_trigger(camera_, &error);
          }
          const auto trigger_error = error == nullptr ? std::string{} :
            error_text(error, "software trigger");
          if (!trigger_error.empty()) {throw std::runtime_error(trigger_error);}
        }
        auto * buffer = arv_stream_timeout_pop_buffer(
          stream_, static_cast<std::uint64_t>(config_.image_timeout_ms) * 1000U);
        if (buffer == nullptr) {throw std::runtime_error("image timeout");}
        // Free-running preview streams can accumulate completed buffers while the
        // browser requests fewer frames than the camera produces. Discard every
        // older completed buffer and process only the newest one available now.
        if (!software_for_request) {
          for (int index = 1; index < config_.buffer_count; ++index) {
            auto * newer = arv_stream_try_pop_buffer(stream_);
            if (newer == nullptr) {break;}
            arv_stream_push_buffer(stream_, buffer);
            buffer = newer;
          }
        }
        if (arv_buffer_get_status(buffer) != ARV_BUFFER_STATUS_SUCCESS) {
          const auto status = arv_buffer_get_status(buffer);
          arv_stream_push_buffer(stream_, buffer);
          incomplete_frames_.fetch_add(1);
          throw std::runtime_error(
                  "incomplete image buffer: " + buffer_status_name(status) + " (status " +
                  std::to_string(status) + ")");
        }
        cv::Mat image;
        try {image = to_bgr(buffer);} catch (...) {
          arv_stream_push_buffer(stream_, buffer);
          throw;
        }
        if (request->completion) {
          request->completion->device_timestamp_ns = arv_buffer_get_timestamp(buffer);
          request->completion->synchronized = ptp_action_ && !request->force_software_trigger;
          request->completion->ptp_offset_ns = request->completion->synchronized ?
            ptp_offset_ns() : 0;
        }
        arv_stream_push_buffer(stream_, buffer);
        if (restore_action_trigger) {
          GError * error = nullptr;
          std::lock_guard<std::mutex> lock(camera_control_mutex_);
          arv_camera_set_trigger(camera_, "Action0", &error);
          throw_on_error(error, "restoring Action0 after temporary software-triggered frame");
          restore_action_trigger = false;
        }
        // Full-resolution one-shot frames use a dedicated lossless compressed
        // topic. Sending the 9 MiB BGR sample directly can stall synchronous
        // DDS writers on hosts with several active camera interfaces.
        endpoint_->publish(
          image, request->stamp, request->capture_id, request->publish_capture_chunks);
        completed_frames_.fetch_add(1);
        if (request->completion) {
          {
            std::lock_guard<std::mutex> completion_lock(request->completion->mutex);
            request->completion->success = true;
            request->completion->done = true;
          }
          request->completion->changed.notify_all();
        }
        std::lock_guard<std::mutex> error_lock(error_mutex_);
        last_error_.clear();
        warning_active_ = false;
      } catch (const std::exception & error) {
        if (restore_action_trigger) {
          GError * restore_error = nullptr;
          {
            std::lock_guard<std::mutex> lock(camera_control_mutex_);
            arv_camera_set_trigger(camera_, "Action0", &restore_error);
          }
          if (restore_error != nullptr) {
            RCLCPP_ERROR(
              node_->get_logger(), "Unable to restore Action0 for %s after failed frame: %s",
              assignment_.sensor_id.c_str(), restore_error->message);
            g_error_free(restore_error);
          }
        }
        if (request->completion) {
          {
            std::lock_guard<std::mutex> completion_lock(request->completion->mutex);
            request->completion->error = error.what();
            request->completion->done = true;
          }
          request->completion->changed.notify_all();
        }
        const auto now = std::chrono::steady_clock::now();
        bool should_log = false;
        {
          std::lock_guard<std::mutex> error_lock(error_mutex_);
          last_error_ = error.what();
          should_log = !warning_active_ || now >= next_warning_;
          if (should_log) {
            warning_active_ = true;
            next_warning_ = now + 5s;
          }
        }
        if (should_log) {
          RCLCPP_WARN(
            node_->get_logger(), "Acquisition failed for %s: %s",
            assignment_.sensor_id.c_str(), error.what());
        }
      }
      {
        std::lock_guard<std::mutex> request_lock(request_mutex_);
        request_active_ = false;
      }
    }
  }

  void add_float_feature(
    std::vector<vixel_interfaces::msg::CameraFeature> & output,
    const std::string & name, const std::string & unit)
  {
    GError * error = nullptr;
    if (!arv_camera_is_feature_available(camera_, name.c_str(), &error)) {
      if (error != nullptr) {g_error_free(error);}
      return;
    }
    vixel_interfaces::msg::CameraFeature feature;
    feature.name = name;
    feature.value_type = "float";
    feature.readable = true;
    feature.writable = true;
    feature.unit = unit;
    const auto value = arv_camera_get_float(camera_, name.c_str(), &error);
    if (error != nullptr) {g_error_free(error); return;}
    double minimum = 0.0, maximum = 0.0;
    arv_camera_get_float_bounds(camera_, name.c_str(), &minimum, &maximum, &error);
    if (error != nullptr) {g_error_free(error); return;}
    feature.current_value_json = std::to_string(value);
    feature.minimum_value_json = std::to_string(minimum);
    feature.maximum_value_json = std::to_string(maximum);
    output.push_back(feature);
  }

  void add_enum_feature(
    std::vector<vixel_interfaces::msg::CameraFeature> & output, const std::string & name)
  {
    GError * error = nullptr;
    if (!arv_camera_is_feature_available(camera_, name.c_str(), &error)) {
      if (error != nullptr) {g_error_free(error);}
      return;
    }
    vixel_interfaces::msg::CameraFeature feature;
    feature.name = name;
    feature.value_type = "enum";
    feature.readable = true;
    feature.writable = true;
    const auto value = arv_camera_get_string(camera_, name.c_str(), &error);
    if (error != nullptr) {g_error_free(error); return;}
    feature.current_value_json = "\"" + json_escape(value_or_empty(value)) + "\"";
    guint count = 0;
    const auto values = arv_camera_dup_available_enumerations_as_strings(
      camera_, name.c_str(), &count, &error);
    if (error == nullptr && values != nullptr) {
      for (guint index = 0; index < count; ++index) {
        feature.enum_values.push_back(value_or_empty(values[index]));
      }
      g_free(values);
    } else if (error != nullptr) {g_error_free(error);}
    output.push_back(feature);
  }

  rclcpp::Node * node_;
  Assignment assignment_;
  DeviceRecord record_;
  NetworkConfig network_;
  GenicamConfig config_;
  std::unique_ptr<CameraEndpoint> endpoint_;
  ArvCamera * camera_{nullptr};
  ArvStream * stream_{nullptr};
  std::thread worker_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<bool> ready_{false};
  std::atomic<std::uint64_t> completed_frames_{0};
  std::atomic<std::uint64_t> incomplete_frames_{0};
  bool software_trigger_{false};
  bool ptp_action_{false};
  bool action_trigger_armed_{false};
  std::string ptp_enable_feature_;
  std::string ptp_status_feature_;
  std::string ptp_offset_feature_;
  std::string ptp_latch_feature_;
  std::string ptp_latch_value_feature_;
  std::chrono::steady_clock::time_point next_ptp_offset_probe_{};
  std::string ptp_capability_detail_{"PTP not requested"};
  std::string device_version_;
  std::mutex camera_control_mutex_;
  bool transfer_start_required_{false};
  std::mutex request_mutex_;
  std::condition_variable request_cv_;
  std::optional<FrameRequest> pending_request_;
  bool request_active_{false};
  std::chrono::steady_clock::time_point next_preview_{};
  std::chrono::steady_clock::time_point next_warning_{};
  mutable std::mutex error_mutex_;
  bool warning_active_{false};
  std::string last_error_;
  std::string applied_settings_{"{}"};
};

class GenicamProvider : public rclcpp::Node
{
public:
  GenicamProvider()
  : rclcpp::Node("genicam_provider", "/vixel/providers/genicam")
  {
    const auto machine_file = declare_parameter<std::string>(
      "machine_file", "/etc/vixel/machine.yaml");
    config_ = load_genicam_config(machine_file);
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    observation_publisher_ = create_publisher<vixel_interfaces::msg::SensorObservationArray>(
      "observations", state_qos);
    status_publisher_ = create_publisher<vixel_interfaces::msg::SensorArray>("status", state_qos);
    group_publisher_ = create_publisher<vixel_interfaces::msg::SyncGroupArray>(
      "group_status", state_qos);
    assignment_subscription_ = create_subscription<vixel_interfaces::msg::ProviderAssignmentArray>(
      "assignments", state_qos,
      std::bind(&GenicamProvider::assignments_callback, this, std::placeholders::_1));
    provision_service_ = create_service<vixel_interfaces::srv::ProvisionSensor>(
      "provision", std::bind(
        &GenicamProvider::provision_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    capture_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    capture_service_ = create_service<vixel_interfaces::srv::ProviderCapture>(
      "capture", std::bind(
        &GenicamProvider::capture_callback, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::ServicesQoS(), capture_callback_group_);
    feature_service_ = create_service<vixel_interfaces::srv::GetCameraFeatures>(
      "features", std::bind(
        &GenicamProvider::features_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    startup_timer_ = create_wall_timer(1ms, [this]() {
        startup_timer_->cancel();
        refresh_devices();
      });
    discovery_timer_ = create_wall_timer(
      std::chrono::milliseconds(config_.discovery_period_ms),
      std::bind(&GenicamProvider::refresh_devices, this));
    preview_timer_ = create_wall_timer(20ms, std::bind(&GenicamProvider::preview_tick, this));
    status_timer_ = create_wall_timer(1s, std::bind(&GenicamProvider::publish_status, this));
    RCLCPP_INFO(
      get_logger(), "Generic GenICam provider started with Aravis %s and config %s",
      ARAVIS_VERSION, machine_file.c_str());
  }

  ~GenicamProvider() override
  {
    startup_timer_.reset();
    discovery_timer_.reset();
    preview_timer_.reset();
    status_timer_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    sessions_.clear();
    arv_shutdown();
  }

private:
  bool vendor_allowed(const std::string & vendor) const
  {
    if (config_.vendor_allowlist.empty()) {return true;}
    const auto candidate = lower(vendor);
    return std::any_of(
      config_.vendor_allowlist.begin(), config_.vendor_allowlist.end(),
      [&candidate](const auto & allowed) {
        return candidate.find(lower(allowed)) != std::string::npos;
      });
  }

  void collect_current_devices(
    const std::string & interface_name, std::map<std::string, DeviceRecord> & output)
  {
    arv_update_device_list();
    const auto count = arv_get_n_devices();
    for (unsigned int index = 0; index < count; ++index) {
      DeviceRecord record;
      record.device_id = value_or_empty(arv_get_device_id(index));
      record.physical_id = value_or_empty(arv_get_device_physical_id(index));
      record.address = value_or_empty(arv_get_device_address(index));
      record.vendor = value_or_empty(arv_get_device_vendor(index));
      record.model = value_or_empty(arv_get_device_model(index));
      record.serial = value_or_empty(arv_get_device_serial_nbr(index));
      record.protocol = value_or_empty(arv_get_device_protocol(index));
      record.interface_name = interface_name;
      if (!record.serial.empty() && vendor_allowed(record.vendor)) {
        const auto key = record.vendor + "\n" + record.serial;
        if (record.interface_name.empty() && output.count(key) != 0) {
          record.interface_name = output.at(key).interface_name;
        }
        output[key] = std::move(record);
      }
    }
  }

  DeviceRecord wait_for_device_address(
    const std::string & serial, const std::string & interface_name,
    const std::string & target_address, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      std::map<std::string, DeviceRecord> discovered;
      arv_gv_interface_set_discovery_interface_name(interface_name.c_str());
      collect_current_devices(interface_name, discovered);
      for (const auto & item : discovered) {
        if (item.second.serial == serial && item.second.address == target_address) {
          return item.second;
        }
      }
      std::this_thread::sleep_for(200ms);
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error(
            "camera did not appear at " + target_address + " on " + interface_name +
            " after GVCP ForceIP");
  }

  void refresh_devices()
  {
    const auto context = get_node_base_interface()->get_context();
    if (!rclcpp::ok(context)) {return;}
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {return;}
    std::map<std::string, DeviceRecord> discovered;
    const auto discovery_started = std::chrono::steady_clock::now();
    try {
      std::lock_guard<std::mutex> aravis_lock(aravis_mutex_);
      arv_enable_interface("GigEVision");
      arv_enable_interface("USB3Vision");
      arv_gv_interface_set_discovery_interface_name(nullptr);
      collect_current_devices("", discovered);
      std::set<std::string> represented_interfaces;
      for (auto & device : discovered) {
        for (const auto & network : config_.networks) {
          if (address_in_cidr(device.second.address, network.second.host_cidr)) {
            device.second.interface_name = network.second.interface;
            represented_interfaces.insert(network.second.interface);
            break;
          }
        }
      }
      // A correctly provisioned camera is already mapped from its subnet.
      // Only scan carrier-up ports that have no mapped camera; those are the
      // ports where a new/off-subnet camera needs interface-local discovery.
      for (const auto & item : config_.networks) {
        if (!interface_has_carrier(item.second.interface) ||
          represented_interfaces.count(item.second.interface) != 0)
        {
          continue;
        }
        const auto raw_devices = gvcp_discover(
          item.second.interface, host_address_for_cidr(item.second.host_cidr), 300ms);
        for (const auto & record : raw_devices) {
          if (vendor_allowed(record.vendor)) {
            discovered[record.vendor + "\n" + record.serial] = record;
          }
        }
        // Keep Aravis' interface-local scan as a fallback for devices whose
        // discovery acknowledgement does not use the standard GVCP layout.
        if (raw_devices.empty()) {
          arv_gv_interface_set_discovery_interface_name(item.second.interface.c_str());
          collect_current_devices(item.second.interface, discovered);
        }
      }
    } catch (const std::exception & error) {
      last_system_error_ = error.what();
    }
    if (!rclcpp::ok(context)) {return;}
    records_.clear();
    vixel_interfaces::msg::SensorObservationArray message;
    message.header.stamp = now();
    for (auto & item : discovered) {
      auto & record = item.second;
      records_[record.serial] = record;
      vixel_interfaces::msg::SensorObservation observation;
      observation.stamp = message.header.stamp;
      observation.provider = "genicam";
      observation.candidate_id = "camera_" + ros_slug(record.vendor) + "_" +
        ros_slug(record.serial);
      observation.kind = "camera";
      observation.vendor = record.vendor;
      observation.model = record.model;
      observation.serial = record.serial;
      observation.mac_address = normalized_mac(record.physical_id);
      observation.transport = record.protocol;
      observation.interface_name = record.interface_name;
      observation.current_address = record.address;
      observation.capabilities = {
        "image", "compressed_preview", "genicam", "camera_settings",
        "software_capture"};
      message.observations.push_back(observation);
    }
    try {
      observation_publisher_->publish(message);
    } catch (const rclcpp::exceptions::RCLError &) {
      if (!rclcpp::ok(context)) {return;}
      throw;
    }
    if (first_discovery_ || discovered.size() != last_discovered_count_) {
      RCLCPP_INFO(
        get_logger(), "GenICam discovery found %zu camera(s) in %.3fs",
        discovered.size(), std::chrono::duration<double>(
          std::chrono::steady_clock::now() - discovery_started).count());
      first_discovery_ = false;
      last_discovered_count_ = discovered.size();
    }
    reconcile_sessions();
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

  void reconcile_sessions()
  {
    for (auto iterator = sessions_.begin(); iterator != sessions_.end();) {
      const auto assignment = assignments_.find(iterator->first);
      const bool keep = assignment != assignments_.end() && assignment->second.enabled &&
        assignment->second.operating_mode != "idle" &&
        assignment->second.operating_mode == iterator->second->assignment().operating_mode &&
        assignment->second.provider_settings_json ==
        iterator->second->assignment().provider_settings_json &&
        assignment->second.preview_rate_hz ==
        iterator->second->assignment().preview_rate_hz &&
        assignment->second.calibration_url ==
        iterator->second->assignment().calibration_url &&
        assignment->second.network_id == iterator->second->assignment().network_id &&
        records_.count(assignment->second.serial) != 0;
      if (!keep) {iterator = sessions_.erase(iterator);} else {++iterator;}
    }
    for (const auto & item : assignments_) {
      const auto & assignment = item.second;
      if (!assignment.enabled || assignment.operating_mode == "idle" ||
        sessions_.count(item.first) != 0)
      {
        continue;
      }
      const auto record = records_.find(assignment.serial);
      const auto network = config_.networks.find(assignment.network_id);
      if (network == config_.networks.end()) {continue;}
      const bool address_mismatch = record == records_.end() ||
        (!assignment.assigned_address.empty() &&
        record->second.address != assignment.assigned_address) ||
        (!record->second.interface_name.empty() &&
        record->second.interface_name != network->second.interface);
      if (address_mismatch && !assignment.assigned_address.empty() &&
        !assignment.mac_address.empty() && interface_has_carrier(network->second.interface))
      {
        const auto now = std::chrono::steady_clock::now();
        const auto previous = last_force_ip_attempt_.find(item.first);
        if (previous == last_force_ip_attempt_.end() || now - previous->second >= 10s) {
          last_force_ip_attempt_[item.first] = now;
          try {
            const auto subnet_mask = subnet_mask_for_cidr(network->second.host_cidr);
            const auto host_address = host_address_for_cidr(network->second.host_cidr);
            const auto broadcast_address = broadcast_address_for_cidr(network->second.host_cidr);
            RCLCPP_INFO(
              get_logger(), "Restoring %s by GVCP ForceIP on %s to %s",
              item.first.c_str(), network->second.interface.c_str(),
              assignment.assigned_address.c_str());
            gvcp_force_ip(
              network->second.interface, host_address, broadcast_address,
              assignment.mac_address,
              assignment.assigned_address, subnet_mask, network->second.gateway);
            initialization_errors_[item.first] =
              "restoring assigned address " + assignment.assigned_address;
          } catch (const std::exception & error) {
            initialization_errors_[item.first] = error.what();
            RCLCPP_WARN(
              get_logger(), "Unable to restore %s with GVCP ForceIP: %s",
              item.first.c_str(), error.what());
          }
        }
        continue;
      }
      if (record == records_.end()) {continue;}
      try {
        std::lock_guard<std::mutex> aravis_lock(aravis_mutex_);
        arv_gv_interface_set_discovery_interface_name(network->second.interface.c_str());
        auto session = std::make_unique<CameraSession>(
          this, assignment, record->second, network->second, config_);
        session->initialize();
        sessions_[item.first] = std::move(session);
        initialization_errors_.erase(item.first);
        last_force_ip_attempt_.erase(item.first);
        RCLCPP_INFO(
          get_logger(), "Started %s (%s %s) at %s with generic GenICam backend",
          item.first.c_str(), record->second.vendor.c_str(), record->second.model.c_str(),
          record->second.address.c_str());
      } catch (const std::exception & error) {
        initialization_errors_[item.first] = error.what();
        RCLCPP_ERROR(
          get_logger(), "Failed to start %s with generic GenICam backend: %s",
          item.first.c_str(), error.what());
      }
    }
    arv_gv_interface_set_discovery_interface_name(nullptr);
  }

  void preview_tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto tick = std::chrono::steady_clock::now();
    const auto stamp = now();
    for (auto & item : sessions_) {
      if (item.second->preview_due(tick)) {
        item.second->request_frame(stamp);
      }
    }
  }

  void publish_status()
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {return;}
    vixel_interfaces::msg::SensorArray status;
    status.header.stamp = now();
    status.generation = ++generation_;
    for (const auto & item : assignments_) {
      const auto session = sessions_.find(item.first);
      if (session != sessions_.end()) {
        status.sensors.push_back(session->second->status());
        continue;
      }
      Sensor sensor;
      sensor.stamp = status.header.stamp;
      sensor.sensor_id = item.first;
      sensor.provider = "genicam";
      sensor.kind = "camera";
      sensor.serial = item.second.serial;
      sensor.mac_address = item.second.mac_address;
      sensor.enrolled = true;
      sensor.enabled = item.second.enabled;
      sensor.online = records_.count(item.second.serial) != 0;
      sensor.lifecycle_state = item.second.operating_mode == "idle" ? "idle" : "offline";
      sensor.topic_base = "/vixel/sensors/" + item.first;
      sensor.network_id = item.second.network_id;
      sensor.assigned_address = item.second.assigned_address;
      sensor.sync_group = item.second.sync_group;
      sensor.operating_mode = item.second.operating_mode;
      sensor.capabilities = {
        "image", "compressed_preview", "genicam", "camera_settings",
        "software_capture", "ptp_capability_unknown"};
      const auto error = initialization_errors_.find(item.first);
      if (error != initialization_errors_.end()) {sensor.last_error = error->second;}
      status.sensors.push_back(sensor);
    }
    status_publisher_->publish(status);
    publish_group_status(status.header.stamp);
  }

  void publish_group_status(const builtin_interfaces::msg::Time & stamp)
  {
    std::map<std::string, SyncGroup> groups;
    std::map<std::string, std::vector<std::string>> locking_members;
    std::map<std::string, std::vector<std::string>> unsupported_members;
    for (const auto & item : assignments_) {
      const auto & assignment = item.second;
      if (assignment.sync_group.empty()) {continue;}
      auto & group = groups[assignment.sync_group];
      group.stamp = stamp;
      group.group_id = assignment.sync_group;
      group.provider = "genicam";
      group.missing_policy = assignment.group_missing_policy;
      group.trigger_source = assignment.group_trigger_source;
      group.operating_mode = assignment.operating_mode;
      group.preview_rate_hz = assignment.preview_rate_hz;
      group.preferred_master_id = assignment.preferred_master_id;
      group.member_ids.push_back(item.first);
      const auto session = sessions_.find(item.first);
      if (session != sessions_.end() && session->second->ready()) {
        group.online_member_ids.push_back(item.first);
        if (session->second->ptp_action()) {
          if (session->second->ptp_locked()) {
            group.synchronized_member_ids.push_back(item.first);
            const auto offset = static_cast<std::int64_t>(
              std::llabs(session->second->ptp_offset_ns()));
            group.max_ptp_offset_ns = std::max<std::int64_t>(
              group.max_ptp_offset_ns, offset);
          } else {
            group.locking_member_ids.push_back(item.first);
            locking_members[assignment.sync_group].push_back(item.first);
          }
        } else {
          group.unsynchronized_member_ids.push_back(item.first);
          unsupported_members[assignment.sync_group].push_back(item.first);
        }
      } else {
        group.missing_member_ids.push_back(item.first);
      }
    }
    vixel_interfaces::msg::SyncGroupArray result;
    result.header.stamp = stamp;
    result.generation = generation_;
    for (auto & item : groups) {
      auto & group = item.second;
      group.synchronization_method = !group.locking_member_ids.empty() ?
        "ptp_relocking" : group.unsynchronized_member_ids.empty() ?
        "ptp_scheduled_action" : "mixed";
      group.ptp_ready = !group.online_member_ids.empty() &&
        group.locking_member_ids.empty() && group.unsynchronized_member_ids.empty();
      const bool members_ready = group.operating_mode == "idle" ||
        group.missing_member_ids.empty() ||
        (group.missing_policy == "degraded" && !group.online_member_ids.empty());
      group.ready = members_ready && group.locking_member_ids.empty();
      if (!group.locking_member_ids.empty() ||
        (group.operating_mode == "capture" && !group.unsynchronized_member_ids.empty()))
      {
        std::ostringstream notice;
        const auto & locking = locking_members[group.group_id];
        const auto & unsupported = unsupported_members[group.group_id];
        if (!locking.empty()) {
          notice << ptp_lock_status(locking);
        }
        if (!unsupported.empty()) {
          if (!locking.empty()) {notice << ". ";}
          notice << "Member(s) without usable PTP scheduled actions: ";
          notice << joined(unsupported);
          notice << "; they remain unsynchronized";
        }
        group.last_error = notice.str();
      }
      result.groups.push_back(group);
    }
    group_publisher_->publish(result);
  }

  void provision_callback(
    const std::shared_ptr<vixel_interfaces::srv::ProvisionSensor::Request> request,
    std::shared_ptr<vixel_interfaces::srv::ProvisionSensor::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      const auto record_iterator = records_.find(request->serial);
      const auto network = config_.networks.find(request->network_id);
      if (record_iterator == records_.end()) {
        throw std::runtime_error("camera is not discoverable");
      }
      if (network == config_.networks.end()) {
        throw std::runtime_error("unknown managed network " + request->network_id);
      }
      auto record = record_iterator->second;
      std::lock_guard<std::mutex> aravis_lock(aravis_mutex_);
      arv_gv_interface_set_discovery_interface_name(network->second.interface.c_str());
      if (record.address != request->target_address) {
        if (request->mac_address.empty()) {
          throw std::runtime_error("camera MAC address is required for initial provisioning");
        }
        RCLCPP_INFO(
          get_logger(), "Temporarily assigning %s (%s) to %s on %s before provisioning",
          request->candidate_id.c_str(), request->mac_address.c_str(),
          request->target_address.c_str(), network->second.interface.c_str());
        gvcp_force_ip(
          network->second.interface,
          host_address_for_cidr(network->second.host_cidr),
          broadcast_address_for_cidr(network->second.host_cidr),
          request->mac_address, request->target_address, request->subnet_mask,
          request->gateway);
        record = wait_for_device_address(
          request->serial, network->second.interface, request->target_address, 8s);
        RCLCPP_INFO(
          get_logger(), "Rediscovered %s at temporary runtime address %s",
          request->candidate_id.c_str(), record.address.c_str());
      }
      records_[request->serial] = record;
      response->success = true;
      response->message = "temporary runtime address assigned; persistent camera settings preserved";
      response->current_address = request->target_address;
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
    }
    arv_gv_interface_set_discovery_interface_name(nullptr);
  }

  void capture_callback(
    const std::shared_ptr<vixel_interfaces::srv::ProviderCapture::Request> request,
    std::shared_ptr<vixel_interfaces::srv::ProviderCapture::Response> response)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto lead = std::chrono::milliseconds(config_.ptp_action_lead_time_ms);
    const auto release_at = std::chrono::steady_clock::now() + lead;
    response->capture_id = request->request_id.empty() ?
      "group_" + std::to_string(++capture_sequence_) : request->request_id;
    std::vector<std::pair<
      std::string, std::shared_ptr<CameraSession::FrameCompletion>>> completions;
    std::map<std::string, NetworkConfig> action_networks;
    std::map<std::string, bool> request_ptp_locked;
    std::optional<std::uint64_t> action_time;
    std::optional<std::uint64_t> minimum_ptp_time;
    std::optional<std::uint64_t> maximum_ptp_time;
    std::vector<std::string> locking_member_ids;
    for (const auto & sensor_id : request->member_ids) {
      const auto session = sessions_.find(sensor_id);
      if (session == sessions_.end() || !session->second->ready() ||
        !session->second->ptp_action()) {continue;}
      const bool locked = session->second->ptp_locked();
      request_ptp_locked[sensor_id] = locked;
      if (!locked) {
        locking_member_ids.push_back(sensor_id);
        continue;
      }
      const auto current_ptp_time = session->second->ptp_time_ns();
      minimum_ptp_time = minimum_ptp_time ?
        std::min(*minimum_ptp_time, current_ptp_time) : current_ptp_time;
      maximum_ptp_time = maximum_ptp_time ?
        std::max(*maximum_ptp_time, current_ptp_time) : current_ptp_time;
      action_networks[session->second->network().id] = session->second->network();
    }
    if (!locking_member_ids.empty()) {
      response->success = false;
      response->message = ptp_lock_error(locking_member_ids);
      RCLCPP_WARN(
        get_logger(), "Group %s capture rejected: %s", request->group_id.c_str(),
        response->message.c_str());
      return;
    }
    if (maximum_ptp_time) {
      action_time = *maximum_ptp_time +
        static_cast<std::uint64_t>(config_.ptp_action_lead_time_ms) * 1000000ULL;
      RCLCPP_INFO(
        get_logger(),
        "Scheduling group %s at PTP %llu from camera range %llu..%llu (%llu ns spread, %d ms lead)",
        request->group_id.c_str(),
        static_cast<unsigned long long>(*action_time),
        static_cast<unsigned long long>(*minimum_ptp_time),
        static_cast<unsigned long long>(*maximum_ptp_time),
        static_cast<unsigned long long>(*maximum_ptp_time - *minimum_ptp_time),
        config_.ptp_action_lead_time_ms);
    }
    if (action_time) {
      response->scheduled_time.sec = static_cast<std::int32_t>(*action_time / 1000000000ULL);
      response->scheduled_time.nanosec =
        static_cast<std::uint32_t>(*action_time % 1000000000ULL);
    } else {
      response->scheduled_time = now() + rclcpp::Duration::from_nanoseconds(
        std::chrono::duration_cast<std::chrono::nanoseconds>(lead).count());
    }
    std::uint64_t earliest_dispatch = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t latest_dispatch = 0;
    std::vector<std::string> action_errors;
    std::set<std::string> failed_action_networks;
    if (action_time) {
      for (const auto & item : action_networks) {
        const auto dispatch = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        earliest_dispatch = std::min(earliest_dispatch, dispatch);
        try {
          gvcp_scheduled_action(
            item.second, config_.action_device_key,
            action_group_key(request->group_id), *action_time);
          latest_dispatch = dispatch;
        } catch (const std::exception & error) {
          failed_action_networks.insert(item.first);
          action_errors.push_back(item.second.interface + ": " + error.what());
          RCLCPP_WARN(
            get_logger(), "Scheduled action failed on %s; using software fallback: %s",
            item.second.interface.c_str(), error.what());
        }
      }
    }
    for (const auto & sensor_id : request->member_ids) {
      const auto session = sessions_.find(sensor_id);
      if (session == sessions_.end() || !session->second->ready() ||
        (!session->second->software_trigger() &&
        !session->second->ptp_action()))
      {
        response->missing_sensor_ids.push_back(sensor_id);
      } else {
        const bool force_software = session->second->ptp_action() &&
          (!request_ptp_locked[sensor_id] ||
          failed_action_networks.count(session->second->network().id) != 0);
        const auto session_release = session->second->software_trigger() || force_software ?
          release_at : std::chrono::steady_clock::time_point{};
        auto completion = session->second->request_frame(
          response->scheduled_time, response->capture_id,
          !request->trigger_only, session_release, force_software);
        if (completion) {
          completions.emplace_back(sensor_id, std::move(completion));
        } else {
          response->missing_sensor_ids.push_back(sensor_id);
        }
      }
    }
    lock.unlock();

    for (const auto & item : completions) {
      const auto & completion = item.second;
      std::unique_lock<std::mutex> completion_lock(completion->mutex);
      completion->changed.wait_until(
        completion_lock, release_at, [&completion]() {return completion->armed;});
    }
    const auto deadline = release_at +
      std::chrono::milliseconds(config_.image_timeout_ms + 1000);
    std::uint64_t earliest_trigger = earliest_dispatch;
    std::uint64_t latest_trigger = latest_dispatch;
    std::uint64_t earliest_exposure = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t latest_exposure = 0;
    for (const auto & item : completions) {
      const auto & completion = item.second;
      std::unique_lock<std::mutex> completion_lock(completion->mutex);
      completion->changed.wait_until(
        completion_lock, deadline, [&completion]() {return completion->done;});
      if (completion->done && completion->success) {
        response->participating_sensor_ids.push_back(item.first);
        vixel_interfaces::msg::CameraTiming timing;
        timing.sensor_id = item.first;
        timing.device_timestamp_ns = completion->device_timestamp_ns;
        timing.ptp_offset_ns = completion->ptp_offset_ns;
        timing.synchronized = completion->synchronized;
        response->camera_timings.push_back(timing);
        if (completion->synchronized && completion->device_timestamp_ns != 0) {
          earliest_exposure = std::min(
            earliest_exposure, completion->device_timestamp_ns);
          latest_exposure = std::max(latest_exposure, completion->device_timestamp_ns);
        }
        if (completion->trigger_started_ns != 0) {
          earliest_trigger = std::min(earliest_trigger, completion->trigger_started_ns);
          latest_trigger = std::max(latest_trigger, completion->trigger_started_ns);
        }
      } else {
        response->missing_sensor_ids.push_back(item.first);
      }
    }
    if (latest_trigger != 0 && earliest_trigger != std::numeric_limits<std::uint64_t>::max()) {
      response->trigger_span_ns = latest_trigger - earliest_trigger;
    }
    if (latest_exposure != 0 &&
      earliest_exposure != std::numeric_limits<std::uint64_t>::max())
    {
      response->exposure_skew_ns = latest_exposure - earliest_exposure;
    }
    response->within_tolerance =
      response->exposure_skew_ns <= static_cast<std::uint64_t>(config_.ptp_tolerance_ns);
    response->success = response->missing_sensor_ids.empty() ||
      (request->missing_policy == "degraded" && !response->participating_sensor_ids.empty());
    const auto synchronized_count = std::count_if(
      response->camera_timings.begin(), response->camera_timings.end(),
      [](const auto & timing) {return timing.synchronized;});
    response->message = response->success ?
      std::to_string(synchronized_count) + " PTP-synchronized and " +
      std::to_string(response->camera_timings.size() - synchronized_count) +
      " unsynchronized frame(s) published" : !action_errors.empty() ?
      "scheduled action and software fallback failed: " + action_errors.front() :
      "one or more cameras were unavailable, PTP-unlocked, busy, or failed acquisition";
  }

  void features_callback(
    const std::shared_ptr<vixel_interfaces::srv::GetCameraFeatures::Request> request,
    std::shared_ptr<vixel_interfaces::srv::GetCameraFeatures::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto session = sessions_.find(request->sensor_id);
    if (session == sessions_.end()) {
      response->success = false;
      response->message = "camera must be online to inspect its GenICam features";
      return;
    }
    response->features = session->second->features();
    if (!request->names.empty()) {
      const std::set<std::string> requested(request->names.begin(), request->names.end());
      response->features.erase(
        std::remove_if(
          response->features.begin(), response->features.end(),
          [&requested](const auto & feature) {return requested.count(feature.name) == 0;}),
        response->features.end());
    }
    response->success = true;
    response->message = "standard GenICam features read";
  }

  GenicamConfig config_;
  std::mutex mutex_;
  std::mutex aravis_mutex_;
  std::map<std::string, DeviceRecord> records_;
  std::map<std::string, Assignment> assignments_;
  std::map<std::string, std::unique_ptr<CameraSession>> sessions_;
  std::map<std::string, std::string> initialization_errors_;
  std::map<std::string, std::chrono::steady_clock::time_point> last_force_ip_attempt_;
  std::string last_system_error_;
  std::uint64_t generation_{0};
  std::uint64_t capture_sequence_{0};
  std::size_t last_discovered_count_{0};
  bool first_discovery_{true};
  rclcpp::Publisher<vixel_interfaces::msg::SensorObservationArray>::SharedPtr observation_publisher_;
  rclcpp::Publisher<vixel_interfaces::msg::SensorArray>::SharedPtr status_publisher_;
  rclcpp::Publisher<vixel_interfaces::msg::SyncGroupArray>::SharedPtr group_publisher_;
  rclcpp::Subscription<vixel_interfaces::msg::ProviderAssignmentArray>::SharedPtr assignment_subscription_;
  rclcpp::Service<vixel_interfaces::srv::ProvisionSensor>::SharedPtr provision_service_;
  rclcpp::Service<vixel_interfaces::srv::ProviderCapture>::SharedPtr capture_service_;
  rclcpp::CallbackGroup::SharedPtr capture_callback_group_;
  rclcpp::Service<vixel_interfaces::srv::GetCameraFeatures>::SharedPtr feature_service_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr preview_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace
}  // namespace vixel_genicam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vixel_genicam::GenicamProvider>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
