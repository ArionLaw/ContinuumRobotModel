cmake_minimum_required(VERSION 3.0.2)
project(dvrk_ctr_teleop)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
 CATKIN_DEPENDS dvrk_planning
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/debug_tf.launch
  launch/suj_ecm_rviz.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_python_setup()
