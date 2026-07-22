cmake_minimum_required(VERSION 3.15)
project(vixel_genicam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vixel_aravis_vendor REQUIRED)
find_package(vixel_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(genicam_config STATIC src/GenicamConfig.cpp)
target_include_directories(genicam_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(genicam_config PUBLIC yaml-cpp)

add_executable(genicam_provider src/GenicamProvider.cpp)
target_include_directories(genicam_provider PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(genicam_provider PRIVATE
  genicam_config
  vixel_aravis_vendor::vixel_aravis_vendor
  ${builtin_interfaces_TARGETS}
  ${camera_info_manager_TARGETS}
  ${cv_bridge_TARGETS}
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${vixel_interfaces_TARGETS}
  ${OpenCV_LIBRARIES})

install(TARGETS genicam_provider DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_genicam_config test/test_genicam_config.cpp)
  target_link_libraries(test_genicam_config genicam_config yaml-cpp)
endif()

ament_package()
