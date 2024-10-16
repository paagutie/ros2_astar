cmake_minimum_required(VERSION 3.5)
project(ros2_astar)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()