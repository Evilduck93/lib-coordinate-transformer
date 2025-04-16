cmake_minimum_required(VERSION 3.10)
project(coordinate_transformer)

# Используем стандарт C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

add_library(coordinate_transformer_lib
  src/coordinate_transformer.cpp
)

ament_target_dependencies(coordinate_transformer_lib
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  yaml-cpp
)

install(TARGETS coordinate_transformer_lib
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

