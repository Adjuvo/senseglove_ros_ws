# Copyright (c) 2020 - 2023 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_description
        VERSION 0.0.0
        DESCRIPTION "The senseglove_description to provide descriptive features of the senseglove"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

include(cmake/${PROJECT_NAME}-extras.cmake)

set(XACRO_FILES
        urdf/dk1_left.xacro
        urdf/dk1_right.xacro
        urdf/nova.xacro
        )
build_and_install_xacro_files(${XACRO_FILES})

install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
