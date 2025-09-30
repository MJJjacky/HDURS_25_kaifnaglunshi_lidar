# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "datmo: 3 messages, 0 services")

set(MSG_I_FLAGS "-Idatmo:/home/jacky/apriltag_ws/src/datmo-master/msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(datmo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg" NAME_WE)
add_custom_target(_datmo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "datmo" "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg" ""
)

get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg" NAME_WE)
add_custom_target(_datmo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "datmo" "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg" "geometry_msgs/Vector3:geometry_msgs/TwistWithCovariance:geometry_msgs/Twist:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:nav_msgs/Odometry:std_msgs/Header:geometry_msgs/Point:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg" NAME_WE)
add_custom_target(_datmo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "datmo" "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg" "geometry_msgs/Vector3:geometry_msgs/TwistWithCovariance:geometry_msgs/Twist:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:datmo/Track:nav_msgs/Odometry:std_msgs/Header:geometry_msgs/Point:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/datmo
)
_generate_msg_cpp(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/datmo
)
_generate_msg_cpp(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/datmo
)

### Generating Services

### Generating Module File
_generate_module_cpp(datmo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/datmo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(datmo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(datmo_generate_messages datmo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg" NAME_WE)
add_dependencies(datmo_generate_messages_cpp _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg" NAME_WE)
add_dependencies(datmo_generate_messages_cpp _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg" NAME_WE)
add_dependencies(datmo_generate_messages_cpp _datmo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(datmo_gencpp)
add_dependencies(datmo_gencpp datmo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS datmo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/datmo
)
_generate_msg_eus(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/datmo
)
_generate_msg_eus(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/datmo
)

### Generating Services

### Generating Module File
_generate_module_eus(datmo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/datmo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(datmo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(datmo_generate_messages datmo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg" NAME_WE)
add_dependencies(datmo_generate_messages_eus _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg" NAME_WE)
add_dependencies(datmo_generate_messages_eus _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg" NAME_WE)
add_dependencies(datmo_generate_messages_eus _datmo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(datmo_geneus)
add_dependencies(datmo_geneus datmo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS datmo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/datmo
)
_generate_msg_lisp(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/datmo
)
_generate_msg_lisp(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/datmo
)

### Generating Services

### Generating Module File
_generate_module_lisp(datmo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/datmo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(datmo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(datmo_generate_messages datmo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg" NAME_WE)
add_dependencies(datmo_generate_messages_lisp _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg" NAME_WE)
add_dependencies(datmo_generate_messages_lisp _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg" NAME_WE)
add_dependencies(datmo_generate_messages_lisp _datmo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(datmo_genlisp)
add_dependencies(datmo_genlisp datmo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS datmo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/datmo
)
_generate_msg_nodejs(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/datmo
)
_generate_msg_nodejs(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/datmo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(datmo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/datmo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(datmo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(datmo_generate_messages datmo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg" NAME_WE)
add_dependencies(datmo_generate_messages_nodejs _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg" NAME_WE)
add_dependencies(datmo_generate_messages_nodejs _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg" NAME_WE)
add_dependencies(datmo_generate_messages_nodejs _datmo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(datmo_gennodejs)
add_dependencies(datmo_gennodejs datmo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS datmo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/datmo
)
_generate_msg_py(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/datmo
)
_generate_msg_py(datmo
  "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/datmo
)

### Generating Services

### Generating Module File
_generate_module_py(datmo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/datmo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(datmo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(datmo_generate_messages datmo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/ThetaPost.msg" NAME_WE)
add_dependencies(datmo_generate_messages_py _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/Track.msg" NAME_WE)
add_dependencies(datmo_generate_messages_py _datmo_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacky/apriltag_ws/src/datmo-master/msg/TrackArray.msg" NAME_WE)
add_dependencies(datmo_generate_messages_py _datmo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(datmo_genpy)
add_dependencies(datmo_genpy datmo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS datmo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/datmo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/datmo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(datmo_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(datmo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/datmo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/datmo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(datmo_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(datmo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/datmo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/datmo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(datmo_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(datmo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/datmo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/datmo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(datmo_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(datmo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/datmo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/datmo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/datmo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(datmo_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(datmo_generate_messages_py std_msgs_generate_messages_py)
endif()
