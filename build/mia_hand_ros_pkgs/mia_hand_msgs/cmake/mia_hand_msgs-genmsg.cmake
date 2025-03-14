# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mia_hand_msgs: 4 messages, 2 services")

set(MSG_I_FLAGS "-Imia_hand_msgs:/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mia_hand_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg" NAME_WE)
add_custom_target(_mia_hand_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mia_hand_msgs" "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg" ""
)

get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg" NAME_WE)
add_custom_target(_mia_hand_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mia_hand_msgs" "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg" ""
)

get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg" NAME_WE)
add_custom_target(_mia_hand_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mia_hand_msgs" "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg" ""
)

get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg" NAME_WE)
add_custom_target(_mia_hand_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mia_hand_msgs" "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv" NAME_WE)
add_custom_target(_mia_hand_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mia_hand_msgs" "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv" ""
)

get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv" NAME_WE)
add_custom_target(_mia_hand_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mia_hand_msgs" "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_cpp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_cpp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_cpp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs
)

### Generating Services
_generate_srv_cpp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs
)
_generate_srv_cpp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs
)

### Generating Module File
_generate_module_cpp(mia_hand_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mia_hand_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mia_hand_msgs_generate_messages mia_hand_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_cpp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_cpp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_cpp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_cpp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_cpp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_cpp _mia_hand_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mia_hand_msgs_gencpp)
add_dependencies(mia_hand_msgs_gencpp mia_hand_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mia_hand_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_eus(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_eus(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_eus(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs
)

### Generating Services
_generate_srv_eus(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs
)
_generate_srv_eus(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs
)

### Generating Module File
_generate_module_eus(mia_hand_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mia_hand_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mia_hand_msgs_generate_messages mia_hand_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_eus _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_eus _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_eus _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_eus _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_eus _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_eus _mia_hand_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mia_hand_msgs_geneus)
add_dependencies(mia_hand_msgs_geneus mia_hand_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mia_hand_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_lisp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_lisp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_lisp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs
)

### Generating Services
_generate_srv_lisp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs
)
_generate_srv_lisp(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs
)

### Generating Module File
_generate_module_lisp(mia_hand_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mia_hand_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mia_hand_msgs_generate_messages mia_hand_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_lisp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_lisp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_lisp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_lisp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_lisp _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_lisp _mia_hand_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mia_hand_msgs_genlisp)
add_dependencies(mia_hand_msgs_genlisp mia_hand_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mia_hand_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_nodejs(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_nodejs(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_nodejs(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs
)

### Generating Services
_generate_srv_nodejs(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs
)
_generate_srv_nodejs(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs
)

### Generating Module File
_generate_module_nodejs(mia_hand_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mia_hand_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mia_hand_msgs_generate_messages mia_hand_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_nodejs _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_nodejs _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_nodejs _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_nodejs _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_nodejs _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_nodejs _mia_hand_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mia_hand_msgs_gennodejs)
add_dependencies(mia_hand_msgs_gennodejs mia_hand_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mia_hand_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_py(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_py(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs
)
_generate_msg_py(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs
)

### Generating Services
_generate_srv_py(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs
)
_generate_srv_py(mia_hand_msgs
  "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs
)

### Generating Module File
_generate_module_py(mia_hand_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mia_hand_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mia_hand_msgs_generate_messages mia_hand_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/GraspRef.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_py _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersData.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_py _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/FingersStrainGauges.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_py _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/msg/ComponentStatus.msg" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_py _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/ConnectSerial.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_py _mia_hand_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ozan/group862_mini_project/src/mia_hand_ros_pkgs/mia_hand_msgs/srv/GetMode.srv" NAME_WE)
add_dependencies(mia_hand_msgs_generate_messages_py _mia_hand_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mia_hand_msgs_genpy)
add_dependencies(mia_hand_msgs_genpy mia_hand_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mia_hand_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mia_hand_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mia_hand_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mia_hand_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mia_hand_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mia_hand_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mia_hand_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mia_hand_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mia_hand_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mia_hand_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mia_hand_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
