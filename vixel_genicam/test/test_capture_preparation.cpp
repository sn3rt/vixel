#include "vixel_genicam/CapturePreparation.hpp"

#include <gtest/gtest.h>

TEST(CapturePreparation, PreparedCadenceQuiescesMeteringAfterInitialFrame)
{
  EXPECT_TRUE(vixel_genicam::background_metering_allowed(250, false));
  EXPECT_FALSE(vixel_genicam::background_metering_allowed(250, true));
}

TEST(CapturePreparation, UnreservedCaptureModeContinuesBackgroundMetering)
{
  EXPECT_TRUE(vixel_genicam::background_metering_allowed(0, false));
  EXPECT_TRUE(vixel_genicam::background_metering_allowed(0, true));
}

TEST(CapturePreparation, PreparedCameraMustBePrimedIdleAndArmed)
{
  EXPECT_FALSE(vixel_genicam::capture_preparation_ready(false, false, true, true, true));
  EXPECT_FALSE(vixel_genicam::capture_preparation_ready(false, true, false, true, true));
  EXPECT_FALSE(vixel_genicam::capture_preparation_ready(false, true, true, true, false));
  EXPECT_TRUE(vixel_genicam::capture_preparation_ready(false, true, true, true, true));
}

TEST(CapturePreparation, MissingTriggerArmedFeatureUsesPrimedIdleFallback)
{
  EXPECT_TRUE(vixel_genicam::capture_preparation_ready(false, true, true, false, false));
}

TEST(CapturePreparation, InFlightCaptureDoesNotRevokeLatchedReadiness)
{
  EXPECT_TRUE(vixel_genicam::capture_preparation_ready(true, true, false, true, false));
}
