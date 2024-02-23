# Copyright (c) 2020 - 2023 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_shared_resources
        VERSION 0.0.0
        DESCRIPTION "The senseglove_shared_resources provides custom messages."
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

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

add_service_files(
  FILES
  Calibrate.srv
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
