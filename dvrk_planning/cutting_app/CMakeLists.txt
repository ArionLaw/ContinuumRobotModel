cmake_minimum_required(VERSION 3.0.2)
project(cutting_app)

find_package(catkin REQUIRED COMPONENTS
  dvrk_planning_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
 CATKIN_DEPENDS dvrk_planning dvrk_planning_msgs rospy
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/cutting_modules/teleop_goal_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
