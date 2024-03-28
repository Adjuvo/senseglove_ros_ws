# Copyright (c) 2020 - 2024 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_haptics
        VERSION 0.0.0
        DESCRIPTION "The senseglove_haptics"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    genmsg
    rospy
    senseglove_shared_resources
    std_msgs
    )

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    senseglove_shared_resources
    std_msgs
)

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
