cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME land_plugin_base)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  rclcpp
  rclcpp_action
  nav_msgs
  as2_core
  as2_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
  include/${PROJECT_NAME}
)

ament_package()