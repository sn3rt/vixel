#include "vixel_genicam/GenicamConfig.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

TEST(GenicamConfig, LoadsProviderAndNetworks)
{
  const auto path = std::filesystem::temp_directory_path() / "vixel-genicam-config.yaml";
  std::ofstream(path) << R"(managed_networks:
  link:
    interface: enp1s0
    host_cidr: 192.168.2.1/24
    approved: true
    packet_size: 8192
providers:
  genicam:
    buffer_count: 20
    stream_restart_error_threshold: 12
    stream_restart_window_ms: 45000
    stream_restart_backoff_ms: 60000
    stream_health_log_period_ms: 90000
    encode_queue_depth: 12
    capture_png_compression: 2
    cadence_safety_margin_ms: 12
    imaging:
      frame_rate_hz: 4.0
defaults:
  preview_width: 800
  preview_format: png
  png_compression: 4
  jpeg_quality: 60
)";
  const auto config = vixel_genicam::load_genicam_config(path.string());
  EXPECT_EQ(config.networks.at("link").interface, "enp1s0");
  EXPECT_TRUE(config.networks.at("link").approved);
  EXPECT_EQ(config.networks.at("link").packet_size, 8192);
  EXPECT_EQ(config.networks.at("link").packet_delay, 100000);
  EXPECT_EQ(config.buffer_count, 20);
  EXPECT_EQ(config.stream_restart_error_threshold, 12);
  EXPECT_EQ(config.stream_restart_window_ms, 45000);
  EXPECT_EQ(config.stream_restart_backoff_ms, 60000);
  EXPECT_EQ(config.stream_health_log_period_ms, 90000);
  EXPECT_EQ(config.encode_queue_depth, 12);
  EXPECT_EQ(config.capture_png_compression, 2);
  EXPECT_EQ(config.cadence_safety_margin_ms, 12);
  EXPECT_DOUBLE_EQ(config.imaging.frame_rate_hz, 4.0);
  EXPECT_EQ(config.preview_width, 800);
  EXPECT_EQ(config.preview_format, "png");
  EXPECT_EQ(config.png_compression, 4);
  EXPECT_EQ(config.jpeg_quality, 60);
  std::filesystem::remove(path);
}

TEST(GenicamConfig, LoadsLegacyLucidSettingsDuringMigration)
{
  const auto path = std::filesystem::temp_directory_path() /
    "vixel-genicam-legacy-config.yaml";
  std::ofstream(path) << R"(managed_networks: {}
providers:
  lucid:
    discovery_period_ms: 3210
    software_trigger_lead_time_ms: 12
    image_timeout_ms: 5000
    imaging:
      frame_rate_hz: 3.0
)";
  const auto config = vixel_genicam::load_genicam_config(path.string());
  EXPECT_EQ(config.discovery_period_ms, 3210);
  EXPECT_EQ(config.software_trigger_lead_time_ms, 12);
  EXPECT_EQ(config.image_timeout_ms, 1000);
  EXPECT_DOUBLE_EQ(config.imaging.frame_rate_hz, 3.0);
  std::filesystem::remove(path);
}

TEST(GenicamConfig, ReportsMissingProvider)
{
  const auto path = std::filesystem::temp_directory_path() /
    "vixel-genicam-missing-provider.yaml";
  std::ofstream(path) << R"(managed_networks: {}
providers: {}
)";

  EXPECT_THROW(vixel_genicam::load_genicam_config(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(GenicamConfig, RequiresExplicitBooleanNetworkApproval)
{
  const auto path = std::filesystem::temp_directory_path() /
    "vixel-genicam-approval-config.yaml";
  const auto write_config = [&path](const std::string & approval) {
      std::ofstream(path) << "managed_networks:\n"
        "  link:\n"
        "    interface: enp1s0\n"
        "    host_cidr: 192.168.2.1/24\n" + approval +
        "providers:\n"
        "  genicam: {}\n";
    };

  write_config("");
  EXPECT_THROW(vixel_genicam::load_genicam_config(path.string()), std::runtime_error);
  write_config("    approved: \"false\"\n");
  EXPECT_THROW(vixel_genicam::load_genicam_config(path.string()), std::runtime_error);
  write_config("    approved: false\n");
  const auto config = vixel_genicam::load_genicam_config(path.string());
  EXPECT_FALSE(config.networks.at("link").approved);
  std::filesystem::remove(path);
}
