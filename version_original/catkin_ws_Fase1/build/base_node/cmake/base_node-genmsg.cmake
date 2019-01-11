# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "base_node: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(base_node_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pi/catkin_ws/src/base_node/srv/MVServ.srv" NAME_WE)
add_custom_target(_base_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "base_node" "/home/pi/catkin_ws/src/base_node/srv/MVServ.srv" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/base_node/srv/OverSrv.srv" NAME_WE)
add_custom_target(_base_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "base_node" "/home/pi/catkin_ws/src/base_node/srv/OverSrv.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(base_node
  "/home/pi/catkin_ws/src/base_node/srv/MVServ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/base_node
)
_generate_srv_cpp(base_node
  "/home/pi/catkin_ws/src/base_node/srv/OverSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/base_node
)

### Generating Module File
_generate_module_cpp(base_node
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/base_node
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(base_node_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(base_node_generate_messages base_node_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/base_node/srv/MVServ.srv" NAME_WE)
add_dependencies(base_node_generate_messages_cpp _base_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/base_node/srv/OverSrv.srv" NAME_WE)
add_dependencies(base_node_generate_messages_cpp _base_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(base_node_gencpp)
add_dependencies(base_node_gencpp base_node_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS base_node_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(base_node
  "/home/pi/catkin_ws/src/base_node/srv/MVServ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/base_node
)
_generate_srv_lisp(base_node
  "/home/pi/catkin_ws/src/base_node/srv/OverSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/base_node
)

### Generating Module File
_generate_module_lisp(base_node
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/base_node
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(base_node_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(base_node_generate_messages base_node_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/base_node/srv/MVServ.srv" NAME_WE)
add_dependencies(base_node_generate_messages_lisp _base_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/base_node/srv/OverSrv.srv" NAME_WE)
add_dependencies(base_node_generate_messages_lisp _base_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(base_node_genlisp)
add_dependencies(base_node_genlisp base_node_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS base_node_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(base_node
  "/home/pi/catkin_ws/src/base_node/srv/MVServ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/base_node
)
_generate_srv_py(base_node
  "/home/pi/catkin_ws/src/base_node/srv/OverSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/base_node
)

### Generating Module File
_generate_module_py(base_node
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/base_node
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(base_node_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(base_node_generate_messages base_node_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/base_node/srv/MVServ.srv" NAME_WE)
add_dependencies(base_node_generate_messages_py _base_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/base_node/srv/OverSrv.srv" NAME_WE)
add_dependencies(base_node_generate_messages_py _base_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(base_node_genpy)
add_dependencies(base_node_genpy base_node_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS base_node_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/base_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/base_node
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(base_node_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/base_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/base_node
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(base_node_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/base_node)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/base_node\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/base_node
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(base_node_generate_messages_py std_msgs_generate_messages_py)
