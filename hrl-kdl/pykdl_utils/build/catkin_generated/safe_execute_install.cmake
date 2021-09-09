execute_process(COMMAND "/home/shahao/catkin_ws/src/hrl-kdl/pykdl_utils/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/shahao/catkin_ws/src/hrl-kdl/pykdl_utils/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
