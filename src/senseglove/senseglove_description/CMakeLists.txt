cmake_minimum_required(VERSION 3.0.2)
project(senseglove_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

include(cmake/${PROJECT_NAME}-extras.cmake)

set(XACRO_FILES
        urdf/dk1_left.xacro
        urdf/dk1_right.xacro
        )
build_and_install_xacro_files(${XACRO_FILES})

install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
