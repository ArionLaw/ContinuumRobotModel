cmake_minimum_required(VERSION 3.0.2)
project(dvrk_ctr_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
 CATKIN_DEPENDS dvrk_planning dvrk_planning_ros rospy
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/dvrk_planning_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/suj_ecm_rviz.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

catkin_python_setup()
