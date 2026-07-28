cmake_minimum_required(VERSION 3.15)
project(vixel_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorObservation.msg"
  "msg/SensorObservationArray.msg"
  "msg/Sensor.msg"
  "msg/SensorArray.msg"
  "msg/KnownSensor.msg"
  "msg/KnownSensorArray.msg"
  "msg/SyncGroup.msg"
  "msg/SyncGroupArray.msg"
  "msg/ManagedPort.msg"
  "msg/ManagedPortArray.msg"
  "msg/ProviderAssignment.msg"
  "msg/ProviderAssignmentArray.msg"
  "msg/CameraFeature.msg"
  "msg/CaptureRecord.msg"
  "msg/CaptureRecordArray.msg"
  "msg/CaptureFrameChunk.msg"
  "srv/UpdateSensorMetadata.srv"
  "srv/ForgetSensor.srv"
  "srv/UpdateKnownSensor.srv"
  "srv/PurgeKnownSensor.srv"
  "srv/SetOperatingMode.srv"
  "srv/UpsertSyncGroup.srv"
  "srv/DeleteSyncGroup.srv"
  "srv/CaptureGroup.srv"
  "srv/ProvisionSensor.srv"
  "srv/ProviderCapture.srv"
  "srv/SetPortMode.srv"
  "srv/GetCameraFeatures.srv"
  "action/EnrollSensor.action"
  "action/ResolveSensorPlacement.action"
  "action/RecordCapture.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
