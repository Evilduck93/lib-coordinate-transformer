cmake_minimum_required(VERSION 3.10)
project(coordinate_transformer)

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

# Основная библиотека
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

target_link_libraries(coordinate_transformer_lib
  yaml-cpp::yaml-cpp
)

target_include_directories(coordinate_transformer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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

# Тесты
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_coordinate_transformer
    test/test_coordinate_transformer.cpp
  )

  if(TARGET test_coordinate_transformer)
    target_link_libraries(test_coordinate_transformer
      coordinate_transformer_lib
    )

    ament_target_dependencies(test_coordinate_transformer
      rclcpp
      tf2
      tf2_ros
      tf2_geometry_msgs
      geometry_msgs
      yaml-cpp
    )
  endif()
endif()

ament_package()

