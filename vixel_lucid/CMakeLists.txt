cmake_minimum_required(VERSION 3.15)
project(vixel_lucid)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vixel_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(arena_sdk QUIET)

add_library(lucid_config STATIC src/LucidConfig.cpp)
target_include_directories(lucid_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lucid_config PUBLIC yaml-cpp)

if(arena_sdk_FOUND)
  add_executable(lucid_provider src/LucidProvider.cpp)
  target_include_directories(lucid_provider PRIVATE include ${arena_sdk_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(lucid_provider PRIVATE
    lucid_config
    ${builtin_interfaces_TARGETS}
    ${camera_info_manager_TARGETS}
    ${cv_bridge_TARGETS}
    ${rclcpp_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${vixel_interfaces_TARGETS}
    ${arena_sdk_LIBRARIES}
    ${OpenCV_LIBRARIES})
  install(TARGETS lucid_provider DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "Arena SDK not found; optional vixel_lucid runtime backend will not be built")
endif()
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lucid_config test/test_lucid_config.cpp)
  target_link_libraries(test_lucid_config lucid_config yaml-cpp)
endif()

ament_package()
