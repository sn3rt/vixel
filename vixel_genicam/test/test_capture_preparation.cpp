#include "vixel_genicam/CapturePreparation.hpp"

#include <gtest/gtest.h>

TEST(CapturePreparation, PreparedCadenceQuiescesMeteringAfterInitialFrame)
{
  EXPECT_TRUE(vixel_genicam::background_metering_allowed(true, false));
  EXPECT_FALSE(vixel_genicam::background_metering_allowed(true, true));
}

TEST(CapturePreparation, UnreservedCaptureModeContinuesBackgroundMetering)
{
  EXPECT_TRUE(vixel_genicam::background_metering_allowed(false, false));
  EXPECT_TRUE(vixel_genicam::background_metering_allowed(false, true));
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

TEST(CapturePreparation, UngroupedCameraGetsPrivateMeteringDomain)
{
  EXPECT_EQ(
    vixel_genicam::capture_metering_domain("camera_ids_1", ""),
    "sensor:camera_ids_1");
  EXPECT_EQ(
    vixel_genicam::capture_metering_domain("camera_ids_1", "front"),
    "front");
}
