cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(dvrk_planning_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
)

add_service_files(
  FILES
  ComputeIK.srv
)

add_message_files(
  FILES
  Waypoints.msg 
  TrajectoryStatus.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
