# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rocket: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irocket:/home/pi/catkin_ws/src/rocket/msg;-Istd_msgs:/home/pi/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rocket_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rocket
  "/home/pi/catkin_ws/src/rocket/msg/RosGPS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rocket
)

### Generating Services

### Generating Module File
_generate_module_cpp(rocket
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rocket
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rocket_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rocket_generate_messages rocket_generate_messages_cpp)

# target for backward compatibility
add_custom_target(rocket_gencpp)
add_dependencies(rocket_gencpp rocket_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rocket_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rocket
  "/home/pi/catkin_ws/src/rocket/msg/RosGPS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rocket
)

### Generating Services

### Generating Module File
_generate_module_lisp(rocket
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rocket
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rocket_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rocket_generate_messages rocket_generate_messages_lisp)

# target for backward compatibility
add_custom_target(rocket_genlisp)
add_dependencies(rocket_genlisp rocket_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rocket_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rocket
  "/home/pi/catkin_ws/src/rocket/msg/RosGPS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rocket
)

### Generating Services

### Generating Module File
_generate_module_py(rocket
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rocket
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rocket_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rocket_generate_messages rocket_generate_messages_py)

# target for backward compatibility
add_custom_target(rocket_genpy)
add_dependencies(rocket_genpy rocket_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rocket_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rocket)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rocket
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(rocket_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rocket)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rocket
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(rocket_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rocket)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rocket\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rocket
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(rocket_generate_messages_py std_msgs_generate_messages_py)
