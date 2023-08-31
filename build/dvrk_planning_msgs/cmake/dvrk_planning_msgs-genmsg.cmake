# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dvrk_planning_msgs: 2 messages, 1 services")

set(MSG_I_FLAGS "-Idvrk_planning_msgs:/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dvrk_planning_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg" NAME_WE)
add_custom_target(_dvrk_planning_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvrk_planning_msgs" "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg" NAME_WE)
add_custom_target(_dvrk_planning_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvrk_planning_msgs" "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv" NAME_WE)
add_custom_target(_dvrk_planning_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvrk_planning_msgs" "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv" "std_msgs/Header:geometry_msgs/Vector3:sensor_msgs/JointState:geometry_msgs/TransformStamped:geometry_msgs/Transform:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvrk_planning_msgs
)
_generate_msg_cpp(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Services
_generate_srv_cpp(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Module File
_generate_module_cpp(dvrk_planning_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvrk_planning_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dvrk_planning_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dvrk_planning_msgs_generate_messages dvrk_planning_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_cpp _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_cpp _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_cpp _dvrk_planning_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvrk_planning_msgs_gencpp)
add_dependencies(dvrk_planning_msgs_gencpp dvrk_planning_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvrk_planning_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvrk_planning_msgs
)
_generate_msg_eus(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Services
_generate_srv_eus(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Module File
_generate_module_eus(dvrk_planning_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvrk_planning_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dvrk_planning_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dvrk_planning_msgs_generate_messages dvrk_planning_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_eus _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_eus _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_eus _dvrk_planning_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvrk_planning_msgs_geneus)
add_dependencies(dvrk_planning_msgs_geneus dvrk_planning_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvrk_planning_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvrk_planning_msgs
)
_generate_msg_lisp(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Services
_generate_srv_lisp(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Module File
_generate_module_lisp(dvrk_planning_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvrk_planning_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dvrk_planning_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dvrk_planning_msgs_generate_messages dvrk_planning_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_lisp _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_lisp _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_lisp _dvrk_planning_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvrk_planning_msgs_genlisp)
add_dependencies(dvrk_planning_msgs_genlisp dvrk_planning_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvrk_planning_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvrk_planning_msgs
)
_generate_msg_nodejs(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Services
_generate_srv_nodejs(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Module File
_generate_module_nodejs(dvrk_planning_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvrk_planning_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dvrk_planning_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dvrk_planning_msgs_generate_messages dvrk_planning_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_nodejs _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_nodejs _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_nodejs _dvrk_planning_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvrk_planning_msgs_gennodejs)
add_dependencies(dvrk_planning_msgs_gennodejs dvrk_planning_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvrk_planning_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvrk_planning_msgs
)
_generate_msg_py(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Services
_generate_srv_py(dvrk_planning_msgs
  "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvrk_planning_msgs
)

### Generating Module File
_generate_module_py(dvrk_planning_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvrk_planning_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dvrk_planning_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dvrk_planning_msgs_generate_messages dvrk_planning_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_py _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/msg/TrajectoryStatus.msg" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_py _dvrk_planning_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_msgs/srv/ComputeIK.srv" NAME_WE)
add_dependencies(dvrk_planning_msgs_generate_messages_py _dvrk_planning_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvrk_planning_msgs_genpy)
add_dependencies(dvrk_planning_msgs_genpy dvrk_planning_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvrk_planning_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvrk_planning_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvrk_planning_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dvrk_planning_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(dvrk_planning_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvrk_planning_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvrk_planning_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(dvrk_planning_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(dvrk_planning_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvrk_planning_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvrk_planning_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dvrk_planning_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(dvrk_planning_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvrk_planning_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvrk_planning_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(dvrk_planning_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(dvrk_planning_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvrk_planning_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvrk_planning_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvrk_planning_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dvrk_planning_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(dvrk_planning_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()