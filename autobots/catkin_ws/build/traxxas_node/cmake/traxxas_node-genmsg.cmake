# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "traxxas_node: 2 messages, 0 services")

set(MSG_I_FLAGS "-Itraxxas_node:/home/blue/catkin_ws/src/traxxas_node/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg;-Itraxxas_node:/home/blue/catkin_ws/src/traxxas_node/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(traxxas_node_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannDriveMsg.msg" NAME_WE)
add_custom_target(_traxxas_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traxxas_node" "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannDriveMsg.msg" ""
)

get_filename_component(_filename "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannMonitorMsg.msg" NAME_WE)
add_custom_target(_traxxas_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traxxas_node" "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannMonitorMsg.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(traxxas_node
  "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannDriveMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traxxas_node
)
_generate_msg_cpp(traxxas_node
  "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannMonitorMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traxxas_node
)

### Generating Services

### Generating Module File
_generate_module_cpp(traxxas_node
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traxxas_node
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(traxxas_node_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(traxxas_node_generate_messages traxxas_node_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannDriveMsg.msg" NAME_WE)
add_dependencies(traxxas_node_generate_messages_cpp _traxxas_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannMonitorMsg.msg" NAME_WE)
add_dependencies(traxxas_node_generate_messages_cpp _traxxas_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traxxas_node_gencpp)
add_dependencies(traxxas_node_gencpp traxxas_node_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traxxas_node_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(traxxas_node
  "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannDriveMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traxxas_node
)
_generate_msg_lisp(traxxas_node
  "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannMonitorMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traxxas_node
)

### Generating Services

### Generating Module File
_generate_module_lisp(traxxas_node
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traxxas_node
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(traxxas_node_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(traxxas_node_generate_messages traxxas_node_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannDriveMsg.msg" NAME_WE)
add_dependencies(traxxas_node_generate_messages_lisp _traxxas_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannMonitorMsg.msg" NAME_WE)
add_dependencies(traxxas_node_generate_messages_lisp _traxxas_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traxxas_node_genlisp)
add_dependencies(traxxas_node_genlisp traxxas_node_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traxxas_node_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(traxxas_node
  "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannDriveMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traxxas_node
)
_generate_msg_py(traxxas_node
  "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannMonitorMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traxxas_node
)

### Generating Services

### Generating Module File
_generate_module_py(traxxas_node
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traxxas_node
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(traxxas_node_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(traxxas_node_generate_messages traxxas_node_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannDriveMsg.msg" NAME_WE)
add_dependencies(traxxas_node_generate_messages_py _traxxas_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/blue/catkin_ws/src/traxxas_node/msg/AckermannMonitorMsg.msg" NAME_WE)
add_dependencies(traxxas_node_generate_messages_py _traxxas_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traxxas_node_genpy)
add_dependencies(traxxas_node_genpy traxxas_node_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traxxas_node_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traxxas_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traxxas_node
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(traxxas_node_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(traxxas_node_generate_messages_cpp nav_msgs_generate_messages_cpp)
add_dependencies(traxxas_node_generate_messages_cpp traxxas_node_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traxxas_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traxxas_node
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(traxxas_node_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(traxxas_node_generate_messages_lisp nav_msgs_generate_messages_lisp)
add_dependencies(traxxas_node_generate_messages_lisp traxxas_node_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traxxas_node)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traxxas_node\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traxxas_node
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(traxxas_node_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(traxxas_node_generate_messages_py nav_msgs_generate_messages_py)
add_dependencies(traxxas_node_generate_messages_py traxxas_node_generate_messages_py)
