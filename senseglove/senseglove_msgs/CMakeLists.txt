# Copyright (c) 2020 - 2025 SenseGlove

cmake_minimum_required(VERSION 3.16...3.25)

project(senseglove_msgs
        VERSION 2.0.0
        DESCRIPTION "The senseglove_msgs provides custom messages."
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    control_msgs
    message_generation
    std_msgs
    trajectory_msgs
    geometry_msgs
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
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    control_msgs
    message_runtime
    std_msgs
    trajectory_msgs
    geometry_msgs
)
