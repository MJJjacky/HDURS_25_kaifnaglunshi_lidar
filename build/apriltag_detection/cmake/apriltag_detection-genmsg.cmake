# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "apriltag_detection: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iapriltag_detection:/home/jacky/apriltag_ws/src/apriltag_detection/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(apriltag_detection_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg" NAME_WE)
add_custom_target(_apriltag_detection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "apriltag_detection" "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(apriltag_detection
  "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltag_detection
)

### Generating Services

### Generating Module File
_generate_module_cpp(apriltag_detection
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltag_detection
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(apriltag_detection_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(apriltag_detection_generate_messages apriltag_detection_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg" NAME_WE)
add_dependencies(apriltag_detection_generate_messages_cpp _apriltag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_detection_gencpp)
add_dependencies(apriltag_detection_gencpp apriltag_detection_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_detection_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(apriltag_detection
  "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltag_detection
)

### Generating Services

### Generating Module File
_generate_module_eus(apriltag_detection
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltag_detection
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(apriltag_detection_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(apriltag_detection_generate_messages apriltag_detection_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg" NAME_WE)
add_dependencies(apriltag_detection_generate_messages_eus _apriltag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_detection_geneus)
add_dependencies(apriltag_detection_geneus apriltag_detection_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_detection_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(apriltag_detection
  "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltag_detection
)

### Generating Services

### Generating Module File
_generate_module_lisp(apriltag_detection
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltag_detection
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(apriltag_detection_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(apriltag_detection_generate_messages apriltag_detection_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg" NAME_WE)
add_dependencies(apriltag_detection_generate_messages_lisp _apriltag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_detection_genlisp)
add_dependencies(apriltag_detection_genlisp apriltag_detection_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_detection_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(apriltag_detection
  "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltag_detection
)

### Generating Services

### Generating Module File
_generate_module_nodejs(apriltag_detection
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltag_detection
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(apriltag_detection_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(apriltag_detection_generate_messages apriltag_detection_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg" NAME_WE)
add_dependencies(apriltag_detection_generate_messages_nodejs _apriltag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_detection_gennodejs)
add_dependencies(apriltag_detection_gennodejs apriltag_detection_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_detection_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(apriltag_detection
  "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_detection
)

### Generating Services

### Generating Module File
_generate_module_py(apriltag_detection
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_detection
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(apriltag_detection_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(apriltag_detection_generate_messages apriltag_detection_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/apriltag_detection/msg/AprilTagDistance.msg" NAME_WE)
add_dependencies(apriltag_detection_generate_messages_py _apriltag_detection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_detection_genpy)
add_dependencies(apriltag_detection_genpy apriltag_detection_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_detection_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltag_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltag_detection
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(apriltag_detection_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltag_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltag_detection
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(apriltag_detection_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltag_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltag_detection
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(apriltag_detection_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltag_detection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltag_detection
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(apriltag_detection_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_detection)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_detection\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_detection
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(apriltag_detection_generate_messages_py std_msgs_generate_messages_py)
endif()
