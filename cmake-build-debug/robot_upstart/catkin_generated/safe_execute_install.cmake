execute_process(COMMAND "/home/midearobot/catkin_ws/src/cmake-build-debug/robot_upstart/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/midearobot/catkin_ws/src/cmake-build-debug/robot_upstart/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
