# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ow_msgs: 5 messages, 0 services")

set(MSG_I_FLAGS "-Iow_msgs:/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ow_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg" NAME_WE)
add_custom_target(_ow_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ow_msgs" "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg" "geometry_msgs/Vector3:geometry_msgs/Point"
)

get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg" NAME_WE)
add_custom_target(_ow_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ow_msgs" "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg" "geometry_msgs/Vector3:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg" NAME_WE)
add_custom_target(_ow_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ow_msgs" "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg" ""
)

get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg" NAME_WE)
add_custom_target(_ow_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ow_msgs" "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg" "ow_msgs/Vector"
)

get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg" NAME_WE)
add_custom_target(_ow_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ow_msgs" "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg" "geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:geometry_msgs/Accel:geometry_msgs/Point:geometry_msgs/Wrench:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ow_msgs
)
_generate_msg_cpp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ow_msgs
)
_generate_msg_cpp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ow_msgs
)
_generate_msg_cpp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ow_msgs
)
_generate_msg_cpp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ow_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(ow_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ow_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ow_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ow_msgs_generate_messages ow_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_cpp _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_cpp _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_cpp _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_cpp _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_cpp _ow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ow_msgs_gencpp)
add_dependencies(ow_msgs_gencpp ow_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ow_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ow_msgs
)
_generate_msg_eus(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ow_msgs
)
_generate_msg_eus(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ow_msgs
)
_generate_msg_eus(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ow_msgs
)
_generate_msg_eus(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ow_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(ow_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ow_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ow_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ow_msgs_generate_messages ow_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_eus _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_eus _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_eus _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_eus _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_eus _ow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ow_msgs_geneus)
add_dependencies(ow_msgs_geneus ow_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ow_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ow_msgs
)
_generate_msg_lisp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ow_msgs
)
_generate_msg_lisp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ow_msgs
)
_generate_msg_lisp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ow_msgs
)
_generate_msg_lisp(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ow_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(ow_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ow_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ow_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ow_msgs_generate_messages ow_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_lisp _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_lisp _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_lisp _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_lisp _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_lisp _ow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ow_msgs_genlisp)
add_dependencies(ow_msgs_genlisp ow_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ow_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ow_msgs
)
_generate_msg_nodejs(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ow_msgs
)
_generate_msg_nodejs(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ow_msgs
)
_generate_msg_nodejs(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ow_msgs
)
_generate_msg_nodejs(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ow_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ow_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ow_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ow_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ow_msgs_generate_messages ow_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_nodejs _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_nodejs _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_nodejs _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_nodejs _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_nodejs _ow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ow_msgs_gennodejs)
add_dependencies(ow_msgs_gennodejs ow_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ow_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs
)
_generate_msg_py(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg"
  "${MSG_I_FLAGS}"
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs
)
_generate_msg_py(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs
)
_generate_msg_py(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs
)
_generate_msg_py(ow_msgs
  "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Accel.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(ow_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ow_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ow_msgs_generate_messages ow_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/LinearState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_py _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/AngularState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_py _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/Vector.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_py _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/JointState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_py _ow_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/marco/ros/workspaces/final_project/src/utilities/ow_msgs/msg/CartesianState.msg" NAME_WE)
add_dependencies(ow_msgs_generate_messages_py _ow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ow_msgs_genpy)
add_dependencies(ow_msgs_genpy ow_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ow_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ow_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ow_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ow_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ow_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ow_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ow_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ow_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ow_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ow_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ow_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ow_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ow_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ow_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ow_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ow_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ow_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ow_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ow_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ow_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
