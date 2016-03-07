# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "outdoor_navigation: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ioutdoor_navigation:/home/blue/catkin_ws/src/outdoor_navigation/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Ioutdoor_navigation:/home/blue/catkin_ws/src/outdoor_navigation/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(outdoor_navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/blue/catkin_ws/src/outdoor_navigation/msg/NavFlagMsg.msg" NAME_WE)
add_custom_target(_outdoor_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "outdoor_navigation" "/home/blue/catkin_ws/src/outdoor_navigation/msg/NavFlagMsg.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(outdoor_navigation
  "/home/blue/catkin_ws/src/outdoor_navigation/msg/NavFlagMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/outdoor_navigation
)

### Generating Services

### Generating Module File
_generate_module_cpp(outdoor_navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/outdoor_navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(outdoor_navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(outdoor_navigation_generate_messages outdoor_navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/blue/catkin_ws/src/outdoor_navigation/msg/NavFlagMsg.msg" NAME_WE)
add_dependencies(outdoor_navigation_generate_messages_cpp _outdoor_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(outdoor_navigation_gencpp)
add_dependencies(outdoor_navigation_gencpp outdoor_navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS outdoor_navigation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(outdoor_navigation
  "/home/blue/catkin_ws/src/outdoor_navigation/msg/NavFlagMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/outdoor_navigation
)

### Generating Services

### Generating Module File
_generate_module_lisp(outdoor_navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/outdoor_navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(outdoor_navigation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(outdoor_navigation_generate_messages outdoor_navigation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/blue/catkin_ws/src/outdoor_navigation/msg/NavFlagMsg.msg" NAME_WE)
add_dependencies(outdoor_navigation_generate_messages_lisp _outdoor_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(outdoor_navigation_genlisp)
add_dependencies(outdoor_navigation_genlisp outdoor_navigation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS outdoor_navigation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(outdoor_navigation
  "/home/blue/catkin_ws/src/outdoor_navigation/msg/NavFlagMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/outdoor_navigation
)

### Generating Services

### Generating Module File
_generate_module_py(outdoor_navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/outdoor_navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(outdoor_navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(outdoor_navigation_generate_messages outdoor_navigation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/blue/catkin_ws/src/outdoor_navigation/msg/NavFlagMsg.msg" NAME_WE)
add_dependencies(outdoor_navigation_generate_messages_py _outdoor_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(outdoor_navigation_genpy)
add_dependencies(outdoor_navigation_genpy outdoor_navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS outdoor_navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/outdoor_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/outdoor_navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(outdoor_navigation_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(outdoor_navigation_generate_messages_cpp outdoor_navigation_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/outdoor_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/outdoor_navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(outdoor_navigation_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(outdoor_navigation_generate_messages_lisp outdoor_navigation_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/outdoor_navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/outdoor_navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/outdoor_navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(outdoor_navigation_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(outdoor_navigation_generate_messages_py outdoor_navigation_generate_messages_py)
