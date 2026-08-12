cmake_minimum_required(VERSION 3.15)
project(vixel_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vixel_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(recorder_config STATIC src/RecorderConfig.cpp)
target_include_directories(recorder_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(recorder_config PUBLIC yaml-cpp)

add_executable(capture_recorder src/CaptureRecorder.cpp)
target_include_directories(capture_recorder PRIVATE include)
target_link_libraries(capture_recorder PRIVATE
  recorder_config
  nlohmann_json::nlohmann_json
  yaml-cpp
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vixel_interfaces_TARGETS})

install(TARGETS capture_recorder DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_recorder_config test/test_recorder_config.cpp)
  target_link_libraries(test_recorder_config recorder_config yaml-cpp)
  ament_add_gtest(test_sequence_timing test/test_sequence_timing.cpp)
  target_include_directories(test_sequence_timing PRIVATE include)
endif()

ament_package()
