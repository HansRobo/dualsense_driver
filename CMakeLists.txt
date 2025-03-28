cmake_minimum_required(VERSION 3.5)
project(dualsense_driver)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/
  DESTINATION lib/${PROJECT_NAME})

ament_package()
