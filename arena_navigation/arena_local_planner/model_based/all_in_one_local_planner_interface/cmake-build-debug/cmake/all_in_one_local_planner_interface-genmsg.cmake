# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "all_in_one_local_planner_interface: 0 messages, 4 services")

set(MSG_I_FLAGS "-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(all_in_one_local_planner_interface_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv" NAME_WE)
add_custom_target(_all_in_one_local_planner_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "all_in_one_local_planner_interface" "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv" "geometry_msgs/Twist:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv" NAME_WE)
add_custom_target(_all_in_one_local_planner_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "all_in_one_local_planner_interface" "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv" ""
)

get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv" NAME_WE)
add_custom_target(_all_in_one_local_planner_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "all_in_one_local_planner_interface" "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv" "nav_msgs/Path:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv" NAME_WE)
add_custom_target(_all_in_one_local_planner_interface_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "all_in_one_local_planner_interface" "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv" "nav_msgs/Path:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_cpp(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_cpp(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_cpp(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_in_one_local_planner_interface
)

### Generating Module File
_generate_module_cpp(all_in_one_local_planner_interface
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_in_one_local_planner_interface
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(all_in_one_local_planner_interface_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(all_in_one_local_planner_interface_generate_messages all_in_one_local_planner_interface_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_cpp _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_cpp _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_cpp _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_cpp _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_in_one_local_planner_interface_gencpp)
add_dependencies(all_in_one_local_planner_interface_gencpp all_in_one_local_planner_interface_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_in_one_local_planner_interface_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_eus(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_eus(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_eus(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_in_one_local_planner_interface
)

### Generating Module File
_generate_module_eus(all_in_one_local_planner_interface
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_in_one_local_planner_interface
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(all_in_one_local_planner_interface_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(all_in_one_local_planner_interface_generate_messages all_in_one_local_planner_interface_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_eus _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_eus _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_eus _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_eus _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_in_one_local_planner_interface_geneus)
add_dependencies(all_in_one_local_planner_interface_geneus all_in_one_local_planner_interface_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_in_one_local_planner_interface_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_lisp(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_lisp(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_lisp(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_in_one_local_planner_interface
)

### Generating Module File
_generate_module_lisp(all_in_one_local_planner_interface
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_in_one_local_planner_interface
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(all_in_one_local_planner_interface_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(all_in_one_local_planner_interface_generate_messages all_in_one_local_planner_interface_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_lisp _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_lisp _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_lisp _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_lisp _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_in_one_local_planner_interface_genlisp)
add_dependencies(all_in_one_local_planner_interface_genlisp all_in_one_local_planner_interface_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_in_one_local_planner_interface_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_nodejs(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_nodejs(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_nodejs(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_in_one_local_planner_interface
)

### Generating Module File
_generate_module_nodejs(all_in_one_local_planner_interface
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_in_one_local_planner_interface
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(all_in_one_local_planner_interface_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(all_in_one_local_planner_interface_generate_messages all_in_one_local_planner_interface_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_nodejs _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_nodejs _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_nodejs _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_nodejs _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_in_one_local_planner_interface_gennodejs)
add_dependencies(all_in_one_local_planner_interface_gennodejs all_in_one_local_planner_interface_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_in_one_local_planner_interface_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_py(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_py(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_in_one_local_planner_interface
)
_generate_srv_py(all_in_one_local_planner_interface
  "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_in_one_local_planner_interface
)

### Generating Module File
_generate_module_py(all_in_one_local_planner_interface
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_in_one_local_planner_interface
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(all_in_one_local_planner_interface_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(all_in_one_local_planner_interface_generate_messages all_in_one_local_planner_interface_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmd.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_py _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/ResetCostmap.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_py _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/SetGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_py _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/johannes/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/all_in_one_local_planner_interface/srv/GetVelCmdWithGlobalPlan.srv" NAME_WE)
add_dependencies(all_in_one_local_planner_interface_generate_messages_py _all_in_one_local_planner_interface_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(all_in_one_local_planner_interface_genpy)
add_dependencies(all_in_one_local_planner_interface_genpy all_in_one_local_planner_interface_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS all_in_one_local_planner_interface_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_in_one_local_planner_interface)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/all_in_one_local_planner_interface
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_in_one_local_planner_interface)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/all_in_one_local_planner_interface
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_in_one_local_planner_interface)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/all_in_one_local_planner_interface
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_in_one_local_planner_interface)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/all_in_one_local_planner_interface
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_in_one_local_planner_interface)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_in_one_local_planner_interface\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/all_in_one_local_planner_interface
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(all_in_one_local_planner_interface_generate_messages_py geometry_msgs_generate_messages_py)
endif()
