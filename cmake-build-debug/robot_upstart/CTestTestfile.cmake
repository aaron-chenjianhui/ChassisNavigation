# CMake generated Testfile for 
# Source directory: /home/midearobot/catkin_ws/src/robot_upstart
# Build directory: /home/midearobot/catkin_ws/src/cmake-build-debug/robot_upstart
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_robot_upstart_roslint_package "/home/midearobot/catkin_ws/src/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/midearobot/catkin_ws/src/cmake-build-debug/test_results/robot_upstart/roslint-robot_upstart.xml" "--working-dir" "/home/midearobot/catkin_ws/src/cmake-build-debug/robot_upstart" "--return-code" "/opt/ros/indigo/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/midearobot/catkin_ws/src/cmake-build-debug/test_results/robot_upstart/roslint-robot_upstart.xml make roslint_robot_upstart")
add_test(_ctest_robot_upstart_nosetests_test "/home/midearobot/catkin_ws/src/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/midearobot/catkin_ws/src/cmake-build-debug/test_results/robot_upstart/nosetests-test.xml" "--return-code" "/home/midearobot/Software/clion-2017.3.4/bin/cmake/bin/cmake -E make_directory /home/midearobot/catkin_ws/src/cmake-build-debug/test_results/robot_upstart" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/midearobot/catkin_ws/src/robot_upstart/test --with-xunit --xunit-file=/home/midearobot/catkin_ws/src/cmake-build-debug/test_results/robot_upstart/nosetests-test.xml")
