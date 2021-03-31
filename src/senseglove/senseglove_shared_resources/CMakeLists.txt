cmake_minimum_required(VERSION 3.0.2)
project(senseglove_shared_resources)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    control_msgs
    message_generation
    std_msgs
    trajectory_msgs
)

add_message_files(
    FILES
    FingerDistanceFloats.msg
    FingerDistances.msg
    KinematicsVect3D.msg
    SenseGloveState.msg
)

#add_action_files(
#    DIRECTORY action
#    FILES
#    .action
#)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    control_msgs
    std_msgs
    trajectory_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    control_msgs
    message_runtime
    std_msgs
    trajectory_msgs
)
