# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "senseglove_shared_resources: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isenseglove_shared_resources:/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Icontrol_msgs:/opt/ros/melodic/share/control_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(senseglove_shared_resources_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg" NAME_WE)
add_custom_target(_senseglove_shared_resources_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "senseglove_shared_resources" "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(senseglove_shared_resources
  "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/senseglove_shared_resources
)

### Generating Services

### Generating Module File
_generate_module_cpp(senseglove_shared_resources
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/senseglove_shared_resources
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(senseglove_shared_resources_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(senseglove_shared_resources_generate_messages senseglove_shared_resources_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg" NAME_WE)
add_dependencies(senseglove_shared_resources_generate_messages_cpp _senseglove_shared_resources_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(senseglove_shared_resources_gencpp)
add_dependencies(senseglove_shared_resources_gencpp senseglove_shared_resources_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS senseglove_shared_resources_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(senseglove_shared_resources
  "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/senseglove_shared_resources
)

### Generating Services

### Generating Module File
_generate_module_eus(senseglove_shared_resources
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/senseglove_shared_resources
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(senseglove_shared_resources_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(senseglove_shared_resources_generate_messages senseglove_shared_resources_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg" NAME_WE)
add_dependencies(senseglove_shared_resources_generate_messages_eus _senseglove_shared_resources_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(senseglove_shared_resources_geneus)
add_dependencies(senseglove_shared_resources_geneus senseglove_shared_resources_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS senseglove_shared_resources_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(senseglove_shared_resources
  "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/senseglove_shared_resources
)

### Generating Services

### Generating Module File
_generate_module_lisp(senseglove_shared_resources
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/senseglove_shared_resources
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(senseglove_shared_resources_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(senseglove_shared_resources_generate_messages senseglove_shared_resources_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg" NAME_WE)
add_dependencies(senseglove_shared_resources_generate_messages_lisp _senseglove_shared_resources_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(senseglove_shared_resources_genlisp)
add_dependencies(senseglove_shared_resources_genlisp senseglove_shared_resources_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS senseglove_shared_resources_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(senseglove_shared_resources
  "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/senseglove_shared_resources
)

### Generating Services

### Generating Module File
_generate_module_nodejs(senseglove_shared_resources
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/senseglove_shared_resources
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(senseglove_shared_resources_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(senseglove_shared_resources_generate_messages senseglove_shared_resources_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg" NAME_WE)
add_dependencies(senseglove_shared_resources_generate_messages_nodejs _senseglove_shared_resources_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(senseglove_shared_resources_gennodejs)
add_dependencies(senseglove_shared_resources_gennodejs senseglove_shared_resources_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS senseglove_shared_resources_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(senseglove_shared_resources
  "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/senseglove_shared_resources
)

### Generating Services

### Generating Module File
_generate_module_py(senseglove_shared_resources
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/senseglove_shared_resources
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(senseglove_shared_resources_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(senseglove_shared_resources_generate_messages senseglove_shared_resources_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rogier/SenseGlove_ROS/ros_workspace/src/senseglove/senseglove_shared_resources/msg/senseglove.msg" NAME_WE)
add_dependencies(senseglove_shared_resources_generate_messages_py _senseglove_shared_resources_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(senseglove_shared_resources_genpy)
add_dependencies(senseglove_shared_resources_genpy senseglove_shared_resources_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS senseglove_shared_resources_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/senseglove_shared_resources)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/senseglove_shared_resources
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(senseglove_shared_resources_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET control_msgs_generate_messages_cpp)
  add_dependencies(senseglove_shared_resources_generate_messages_cpp control_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(senseglove_shared_resources_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(senseglove_shared_resources_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/senseglove_shared_resources)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/senseglove_shared_resources
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(senseglove_shared_resources_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET control_msgs_generate_messages_eus)
  add_dependencies(senseglove_shared_resources_generate_messages_eus control_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(senseglove_shared_resources_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(senseglove_shared_resources_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/senseglove_shared_resources)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/senseglove_shared_resources
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(senseglove_shared_resources_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET control_msgs_generate_messages_lisp)
  add_dependencies(senseglove_shared_resources_generate_messages_lisp control_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(senseglove_shared_resources_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(senseglove_shared_resources_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/senseglove_shared_resources)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/senseglove_shared_resources
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(senseglove_shared_resources_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET control_msgs_generate_messages_nodejs)
  add_dependencies(senseglove_shared_resources_generate_messages_nodejs control_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(senseglove_shared_resources_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(senseglove_shared_resources_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/senseglove_shared_resources)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/senseglove_shared_resources\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/senseglove_shared_resources
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(senseglove_shared_resources_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET control_msgs_generate_messages_py)
  add_dependencies(senseglove_shared_resources_generate_messages_py control_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(senseglove_shared_resources_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(senseglove_shared_resources_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
