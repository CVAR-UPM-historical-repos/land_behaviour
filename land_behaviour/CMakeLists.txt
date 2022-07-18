cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME land_behaviour)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  land_plugin_base
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
  std_srvs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_executable(land_behaviour_node src/land_behaviour_node.cpp)
target_include_directories(land_behaviour_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(land_behaviour_node ${PROJECT_DEPENDENCIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS land_behaviour_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()