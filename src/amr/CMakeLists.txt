################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(amr)


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)





################################################################################
# Install
################################################################################
install(DIRECTORY launch models rviz urdf worlds params map scripts
  DESTINATION share/${PROJECT_NAME}/
)
set(EXEC_NAME "ros2_control_center.py")
set(EXEC_NAME2 "ros2_control_center_multi_String.py")
set(EXEC_NAME3 "caster_nmpc.py")
install(PROGRAMS scripts/ros2_control_center.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ros2_control_center_multi_String.py
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/caster_nmpc.py
        DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

ament_package()
