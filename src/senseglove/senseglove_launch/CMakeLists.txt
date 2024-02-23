# Copyright (c) 2020 - 2023 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_launch
        VERSION 0.0.0
        DESCRIPTION "The senseglove_launch provides a clear directory from which one can launch the senseglove."
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(PROGRAMS scripts/startGlove.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})