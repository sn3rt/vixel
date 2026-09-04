#include "vixel_genicam/TriggerSelection.hpp"

#include <gtest/gtest.h>

TEST(TriggerSelection, PrefersFrameStartWhenAvailable)
{
  const auto selectors = vixel_genicam::preferred_trigger_selectors(
    {"ExposureStart", "AcquisitionStart", "FrameStart"});
  ASSERT_EQ(selectors.size(), 3U);
  EXPECT_EQ(selectors[0], "FrameStart");
  EXPECT_EQ(selectors[1], "ExposureStart");
  EXPECT_EQ(selectors[2], "AcquisitionStart");
}

TEST(TriggerSelection, UsesIdsExposureStartBeforeAcquisitionStart)
{
  const auto selectors = vixel_genicam::preferred_trigger_selectors(
    {"AcquisitionStart", "ExposureStart"});
  ASSERT_EQ(selectors.size(), 2U);
  EXPECT_EQ(selectors[0], "ExposureStart");
  EXPECT_EQ(selectors[1], "AcquisitionStart");
}

TEST(TriggerSelection, PreservesAdvertisedSpellingAndUnknownFallbacks)
{
  const auto selectors = vixel_genicam::preferred_trigger_selectors(
    {"vendorStart", "exposurestart"});
  ASSERT_EQ(selectors.size(), 2U);
  EXPECT_EQ(selectors[0], "exposurestart");
  EXPECT_EQ(selectors[1], "vendorStart");
  EXPECT_EQ(
    vixel_genicam::advertised_trigger_value({"SOFTWARE", "Line0"}, "Software"),
    "SOFTWARE");
}
