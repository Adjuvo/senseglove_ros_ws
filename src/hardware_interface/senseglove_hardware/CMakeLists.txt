# Copyright (c) 2020 - 2023 SenseGlove

cmake_minimum_required(VERSION 3.21)

project(senseglove_hardware
        VERSION 0.0.0
        DESCRIPTION "The SenseGlove Hardware for communication with the Device"
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
        urdf
        LIBRARIES ${PROJECT_NAME}
        CFG_EXTRAS
        ${PROJECT_NAME}-extras.cmake
        )

include(cmake/${PROJECT_NAME}-extras.cmake)

include_directories(
        include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ../../SenseGlove_API/include/SenseGlove/Core
        ../../SenseGlove_API/include/SenseGlove/Connect
        )

set(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    # Add compiler flags for coverage instrumentation before defining any targets
    append_coverage_compiler_flags()
endif()

add_library(${PROJECT_NAME}
        include/${PROJECT_NAME}/actuation_mode.h
        include/${PROJECT_NAME}/joint.h
        include/${PROJECT_NAME}/senseglove_robot.h
        include/${PROJECT_NAME}/senseglove_setup.h
        src/joint.cpp
        src/senseglove_robot.cpp
        src/senseglove_setup.cpp
        )

target_link_libraries(${PROJECT_NAME} 
        ${catkin_LIBRARIES} 
        ${SGCORE} 
        ${SGCONNECT} 
        ${SGSERIAL} 
        pthread
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(DIRECTORY SenseCom-1.3.1
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
