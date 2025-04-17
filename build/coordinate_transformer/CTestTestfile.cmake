# CMake generated Testfile for 
# Source directory: /home/user/ros2_ws/src/coordinate_transformer
# Build directory: /home/user/ros2_ws/build/coordinate_transformer
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_coordinate_transformer "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/build/coordinate_transformer/test_results/coordinate_transformer/test_coordinate_transformer.gtest.xml" "--package-name" "coordinate_transformer" "--output-file" "/home/user/ros2_ws/build/coordinate_transformer/ament_cmake_gtest/test_coordinate_transformer.txt" "--command" "/home/user/ros2_ws/build/coordinate_transformer/test_coordinate_transformer" "--gtest_output=xml:/home/user/ros2_ws/build/coordinate_transformer/test_results/coordinate_transformer/test_coordinate_transformer.gtest.xml")
set_tests_properties(test_coordinate_transformer PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/user/ros2_ws/build/coordinate_transformer/test_coordinate_transformer" TIMEOUT "60" WORKING_DIRECTORY "/home/user/ros2_ws/build/coordinate_transformer" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;95;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/user/ros2_ws/src/coordinate_transformer/CMakeLists.txt;60;ament_add_gtest;/home/user/ros2_ws/src/coordinate_transformer/CMakeLists.txt;0;")
subdirs("gtest")
