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
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vixel_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(capture_recorder src/CaptureRecorder.cpp)
target_include_directories(capture_recorder PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(capture_recorder PRIVATE
  nlohmann_json::nlohmann_json
  yaml-cpp
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vixel_interfaces_TARGETS}
  ${OpenCV_LIBRARIES})

install(TARGETS capture_recorder DESTINATION lib/${PROJECT_NAME})

ament_package()
