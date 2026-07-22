#include <vixel_lucid/LucidConfig.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

TEST(LucidConfig, LoadsArbitraryManagedNetworks)
{
  const auto path = std::filesystem::temp_directory_path() / "vixel-lucid-machine.yaml";
  std::ofstream(path) << R"(
schema_version: 1
defaults: {preview_width: 800, jpeg_quality: 75}
managed_networks:
  direct_a: {interface: enp2s0, interface_mac: '02:00:00:00:00:02', host_cidr: 192.168.2.1/24}
  switch_b: {interface: enp10s0, host_cidr: 192.168.3.1/24, packet_delay: 80}
providers:
  lucid:
    model_allowlist: [TRI032S, ATL]
    discovery_period_ms: 750
)";
  const auto config = vixel_lucid::load_lucid_config(path.string());
  EXPECT_EQ(config.networks.size(), 2U);
  EXPECT_EQ(config.preview_width, 800);
  EXPECT_EQ(config.jpeg_quality, 75);
  EXPECT_EQ(config.discovery_period_ms, 750);
  EXPECT_EQ(config.networks.at("direct_a").interface_mac, "02:00:00:00:00:02");
  EXPECT_EQ(config.networks.at("switch_b").packet_delay, 80);
  ASSERT_EQ(config.model_allowlist.size(), 2U);
  EXPECT_EQ(config.model_allowlist.at(0), "TRI032S");
  EXPECT_EQ(config.model_allowlist.at(1), "ATL");
  std::filesystem::remove(path);
}

TEST(LucidConfig, AllowsAllModelsWhenAllowlistIsOmitted)
{
  const auto path = std::filesystem::temp_directory_path() /
    "vixel-lucid-machine-no-model-filter.yaml";
  std::ofstream(path) << R"(
schema_version: 1
managed_networks:
  direct_a: {interface: enp7s0, host_cidr: 192.168.2.1/24}
providers:
  lucid: {}
)";
  const auto config = vixel_lucid::load_lucid_config(path.string());
  EXPECT_TRUE(config.model_allowlist.empty());
  std::filesystem::remove(path);
}
