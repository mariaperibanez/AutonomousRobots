execute_process(COMMAND "/home/arob/catkin_ws/build/unique_id/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/arob/catkin_ws/build/unique_id/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
