# Copyright (c) 2020 - 2023 SenseGlove

cmake_minimum_required(VERSION 3.21)

project(senseglove_hardware_builder
        VERSION 0.0.0
        DESCRIPTION "The senseglove_hardware_builder for building the device in ROS"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

# Set C++ standard and compiler options
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
add_compile_options(-Wall -Wextra -Werror)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        senseglove_hardware
        urdf
        )

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(SGCORE
        NAMES libsgcore.so
        PATHS ../../SenseGlove_API/lib/linux/v20/x86-64/release/
        REQUIRED
        )

find_library(SGCONNECT
        NAMES libsgconnect.so
        PATHS ../../SenseGlove_API/lib/linux/v20/x86-64/release/
        REQUIRED
        )

find_library(SGSERIAL
        NAMES libserial.so
        PATHS ../../SenseGlove_API/lib/linux/v20/x86-64/release/
        REQUIRED
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        roslib
        senseglove_hardware
        urdf
        LIBRARIES ${PROJECT_NAME}
)


include_directories(include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ../../SenseGlove_API/include/SenseGlove/Core
        ../../SenseGlove_API/include/SenseGlove/Connect
        )

add_library(${PROJECT_NAME}
        src/hardware_builder.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${SGCORE}
        ${SGCONNECT}
        ${SGSERIAL}
        yaml-cpp
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
