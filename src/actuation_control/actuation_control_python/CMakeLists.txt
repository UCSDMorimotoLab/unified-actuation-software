cmake_minimum_required (VERSION 3.16)
project (actuation_control_python VERSION 1.3.1)

# look for ROS2
find_package (ament_cmake QUIET)
if (ament_cmake_FOUND)
  find_package (ament_cmake_python REQUIRED)
  ament_python_install_package (actuation_control
    PACKAGE_DIR ${actuation_control_python_SOURCE_DIR}/src/actuation_control)

  file (GLOB actuation_control_SCRIPTS scripts/*.py)
  install (PROGRAMS ${actuation_control_SCRIPTS}
    DESTINATION lib/${PROJECT_NAME})

  ament_package ()
endif (ament_cmake_FOUND)

