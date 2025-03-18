# Copyright (c) 2020 - 2024 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_interaction
        VERSION 2.1.0
        DESCRIPTION "The senseglove_interaction"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    actionlib
    actionlib_msgs
    genmsg
    rospy
    senseglove_shared_resources
    std_msgs
    )

catkin_python_setup()

generate_dynamic_reconfigure_options(
	config/HapticLevels.cfg
)

catkin_package(
    CATKIN_DEPENDS
    senseglove_shared_resources
    std_msgs
)

catkin_install_python(PROGRAMS 
    scripts/senseglove_finger_distance_node
    scripts/senseglove_haptics_node
    src/senseglove_interaction/finger_distance/finger_distance_calibration.py
    src/senseglove_interaction/haptics/haptics_node_simple.py
    src/senseglove_interaction/haptics/haptics_node_dynamic.py
    src/senseglove_interaction/common/imu_tf_broadcaster.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
