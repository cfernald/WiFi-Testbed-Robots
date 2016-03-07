# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(WARNING "Invoking generate_messages() without having added any message or service file before.
You should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")
message(STATUS "proteus3_navigation: 0 messages, 0 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg;-Itraxxas_node:/home/blue/catkin_ws/src/traxxas_node/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(proteus3_navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(proteus3_navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/proteus3_navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(proteus3_navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(proteus3_navigation_generate_messages proteus3_navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(proteus3_navigation_gencpp)
add_dependencies(proteus3_navigation_gencpp proteus3_navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS proteus3_navigation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(proteus3_navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/proteus3_navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(proteus3_navigation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(proteus3_navigation_generate_messages proteus3_navigation_generate_messages_lisp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(proteus3_navigation_genlisp)
add_dependencies(proteus3_navigation_genlisp proteus3_navigation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS proteus3_navigation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(proteus3_navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/proteus3_navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(proteus3_navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(proteus3_navigation_generate_messages proteus3_navigation_generate_messages_py)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(proteus3_navigation_genpy)
add_dependencies(proteus3_navigation_genpy proteus3_navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS proteus3_navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/proteus3_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/proteus3_navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(proteus3_navigation_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(proteus3_navigation_generate_messages_cpp nav_msgs_generate_messages_cpp)
add_dependencies(proteus3_navigation_generate_messages_cpp traxxas_node_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/proteus3_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/proteus3_navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(proteus3_navigation_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(proteus3_navigation_generate_messages_lisp nav_msgs_generate_messages_lisp)
add_dependencies(proteus3_navigation_generate_messages_lisp traxxas_node_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/proteus3_navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/proteus3_navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/proteus3_navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(proteus3_navigation_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(proteus3_navigation_generate_messages_py nav_msgs_generate_messages_py)
add_dependencies(proteus3_navigation_generate_messages_py traxxas_node_generate_messages_py)
