# Copyright (c) 2020 - 2023 SenseGlove

cmake_minimum_required(VERSION 3.21)

project(senseglove_hardware_interface
        VERSION 0.0.0
        DESCRIPTION "The senseglove_hardware_interface for the bridge between ros_control and the senseglove hardware."
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
        controller_manager
        joint_state_controller
        joint_trajectory_controller
        realtime_tools
        roscpp
        senseglove_description
        senseglove_hardware
        senseglove_hardware_builder
        senseglove_shared_resources
        std_msgs
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
        controller_manager
        realtime_tools
        roscpp
        senseglove_description
        senseglove_hardware
        senseglove_hardware_builder
        senseglove_shared_resources
        std_msgs
        urdf
)

set(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

include_directories(include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ../../SenseGlove_API/include/SenseGlove/Core
        ../../SenseGlove_API/include/SenseGlove/Connect
        )

add_executable(${PROJECT_NAME}_node
        src/${PROJECT_NAME}.cpp
        src/${PROJECT_NAME}_node.cpp
        )

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${SGCORE}
    ${SGCONNECT}
    ${SGSERIAL}
    )

# From senseglove_hardware/cmake
ros_enable_rpath(${PROJECT_NAME}_node)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

## Add gtest based cpp test target and link libraries
#if(CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(${PROJECT_NAME}_test
#            test/test_runner.cpp
#            )
#    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
#
#    if(ENABLE_COVERAGE_TESTING)
#        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*")
#        add_code_coverage(
#                NAME coverage_report
#                DEPENDENCIES ${PROJECT_NAME}_test
#        )
#    endif()
#endif()
