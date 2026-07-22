cmake_minimum_required(VERSION 3.15)
project(vixel)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_package()
