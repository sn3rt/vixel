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
    packet_size: 8192
providers:
  genicam:
    buffer_count: 20
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
  EXPECT_EQ(config.networks.at("link").packet_size, 8192);
  EXPECT_EQ(config.buffer_count, 20);
  EXPECT_DOUBLE_EQ(config.imaging.frame_rate_hz, 4.0);
  EXPECT_EQ(config.preview_width, 800);
  EXPECT_EQ(config.preview_format, "png");
  EXPECT_EQ(config.png_compression, 4);
  EXPECT_EQ(config.jpeg_quality, 60);
  std::filesystem::remove(path);
}
