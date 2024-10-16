cmake_minimum_required(VERSION 3.8)
project(astar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
  std_srvs
  sensor_msgs
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_msgs
  path_msgs
  tf2
  OpenCV
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include
  $(rclcpp_INCLUDE_DIRS)
)

#ament_python_install_package(${PROJECT_NAME})

file(GLOB SOURCE_FILES
 src/Astar.cpp
 src/OccMapTransform.cpp
 include/${PROJECT_NAME}/Astar.h
 include/${PROJECT_NAME}/OccMapTransform.h
)

add_executable(${PROJECT_NAME} src/astar_ros_node.cpp ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})


install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

#Install Python scripts
#file(GLOB SCRIPT_FILES scripts/*)
install(PROGRAMS
  #scripts/syncronize_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()