#include "vixel_genicam/SessionLiveness.hpp"

#include <gtest/gtest.h>

TEST(SessionLiveness, RequiresACurrentDiscoveryRecord)
{
  EXPECT_FALSE(vixel_genicam::session_endpoint_matches(
    false, {}, {}, "192.168.2.11", "enp7s0"));
}

TEST(SessionLiveness, RejectsChangedAddressOrInterface)
{
  EXPECT_FALSE(vixel_genicam::session_endpoint_matches(
    true, "169.254.1.2", "enp7s0", "192.168.2.11", "enp7s0"));
  EXPECT_FALSE(vixel_genicam::session_endpoint_matches(
    true, "192.168.2.11", "enp8s0", "192.168.2.11", "enp7s0"));
}

TEST(SessionLiveness, AcceptsAssignedEndpointAndUnknownDiscoveryInterface)
{
  EXPECT_TRUE(vixel_genicam::session_endpoint_matches(
    true, "192.168.2.11", "enp7s0", "192.168.2.11", "enp7s0"));
  EXPECT_TRUE(vixel_genicam::session_endpoint_matches(
    true, "192.168.2.11", {}, "192.168.2.11", "enp7s0"));
}

TEST(SessionLiveness, AllowsAnUnassignedAddress)
{
  EXPECT_TRUE(vixel_genicam::session_endpoint_matches(
    true, "169.254.1.2", "enp7s0", {}, "enp7s0"));
}
