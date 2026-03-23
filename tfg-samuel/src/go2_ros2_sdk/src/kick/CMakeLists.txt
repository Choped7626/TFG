cmake_minimum_required(VERSION 3.5)
project(kick)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(
  DIRECTORY
    scripts
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/kick.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
