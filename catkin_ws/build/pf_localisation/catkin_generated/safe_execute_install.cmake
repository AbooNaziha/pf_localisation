execute_process(COMMAND "/home/ismaila/pf_localisation/catkin_ws/build/pf_localisation/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ismaila/pf_localisation/catkin_ws/build/pf_localisation/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
