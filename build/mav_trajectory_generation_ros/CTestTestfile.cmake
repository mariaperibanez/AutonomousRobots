# CMake generated Testfile for 
# Source directory: /home/arob/catkin_ws/src/mav_trajectory_generation/mav_trajectory_generation_ros
# Build directory: /home/arob/catkin_ws/build/mav_trajectory_generation_ros
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_mav_trajectory_generation_ros_gtest_test_feasibility "/home/arob/catkin_ws/build/mav_trajectory_generation_ros/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/arob/catkin_ws/build/mav_trajectory_generation_ros/test_results/mav_trajectory_generation_ros/gtest-test_feasibility.xml" "--return-code" "/home/arob/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib/mav_trajectory_generation_ros/test_feasibility --gtest_output=xml:/home/arob/catkin_ws/build/mav_trajectory_generation_ros/test_results/mav_trajectory_generation_ros/gtest-test_feasibility.xml")
set_tests_properties(_ctest_mav_trajectory_generation_ros_gtest_test_feasibility PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/arob/catkin_ws/src/mav_trajectory_generation/mav_trajectory_generation_ros/CMakeLists.txt;51;catkin_add_gtest;/home/arob/catkin_ws/src/mav_trajectory_generation/mav_trajectory_generation_ros/CMakeLists.txt;0;")
subdirs("gtest")
