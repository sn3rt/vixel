#include "vixel_genicam/CaptureCadence.hpp"

#include <gtest/gtest.h>

TEST(CaptureCadence, OverlapUsesTheSlowerExposureOrFramePeriod)
{
  const vixel_genicam::CaptureCadence cadence{
    200, 5.5, 180000.0, true, 4, 10};

  EXPECT_EQ(vixel_genicam::minimum_capture_interval_ms(cadence), 192U);
  EXPECT_NE(
    vixel_genicam::capture_cadence_reason(cadence).find("frame period"),
    std::string::npos);
}

TEST(CaptureCadence, NoOverlapIncludesExposureAndReadout)
{
  const vixel_genicam::CaptureCadence cadence{
    500, 4.953511, 201721.44, false, 1, 10};

  EXPECT_EQ(vixel_genicam::minimum_capture_interval_ms(cadence), 414U);
  EXPECT_NE(
    vixel_genicam::capture_cadence_reason(cadence).find("non-overlapped readout"),
    std::string::npos);
}

TEST(CaptureCadence, SequenceIntervalDerivesExposureCeiling)
{
  const vixel_genicam::CaptureCadence overlapping{
    200, 10.0, 0.0, true, 1, 10};
  const vixel_genicam::CaptureCadence non_overlapping{
    200, 10.0, 0.0, false, 1, 10};

  ASSERT_TRUE(vixel_genicam::capture_exposure_limit_us(overlapping));
  EXPECT_DOUBLE_EQ(*vixel_genicam::capture_exposure_limit_us(overlapping), 190000.0);
  ASSERT_TRUE(vixel_genicam::capture_exposure_limit_us(non_overlapping));
  EXPECT_DOUBLE_EQ(*vixel_genicam::capture_exposure_limit_us(non_overlapping), 90000.0);
}

TEST(CaptureCadence, DerivedCeilingDoesNotRoundExactCadenceUpOneMillisecond)
{
  vixel_genicam::CaptureCadence cadence{200, 7.3, 0.0, false, 1, 10};
  cadence.exposure_budget_us = *vixel_genicam::capture_exposure_limit_us(cadence);

  EXPECT_EQ(vixel_genicam::minimum_capture_interval_ms(cadence), 200U);
}

TEST(CaptureCadence, RejectsRateBeyondCameraReadoutCapability)
{
  const vixel_genicam::CaptureCadence cadence{
    200, 4.0, 0.0, true, 1, 10};

  EXPECT_FALSE(vixel_genicam::capture_exposure_limit_us(cadence));
}

TEST(CaptureCadence, UnknownFrameRateDoesNotInventALimit)
{
  const vixel_genicam::CaptureCadence cadence{};

  EXPECT_EQ(vixel_genicam::minimum_capture_interval_ms(cadence), 0U);
}
