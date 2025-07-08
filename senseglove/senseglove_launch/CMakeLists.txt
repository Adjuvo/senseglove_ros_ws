# Copyright (c) 2020 - 2025 SenseGlove

cmake_minimum_required(VERSION 3.16...3.25)

project(senseglove_launch
        VERSION 2.0.0
        DESCRIPTION "The senseglove_launch provides a clear directory from which one can launch the senseglove."
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(PROGRAMS 
        bluetooth_scripts/glove_connect.sh 
        bluetooth_scripts/glove_disconnect.sh
        scripts/startGlove.sh 
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )