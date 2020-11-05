cmake_minimum_required(VERSION 3.0.2)
project(senseglove_hardware_interface)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

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

find_library(SENSEGLOVE
        NAMES libSGCORE.so
        PATHS /usr/local/lib/SenseGlove_API/Core/SGCoreCpp/lib/Linux
        REQUIRED)

find_library(SENSEGLOVE_CONNECT
        NAMES SGConnect.so
        PATHS /usr/local/lib/SenseGlove_API/Core/SGConnect/lib/Linux
        REQUIRED)

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

#if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
#    find_package(code_coverage REQUIRED)
#    # Add compiler flags for coverage instrumentation before defining any targets
#    append_coverage_compiler_flags()
#endif()

include_directories(include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        /usr/local/lib/SenseGlove_API/Core/SGConnect/incl
        /usr/local/lib/SenseGlove_API/Core/SGCoreCpp/incl)

add_executable(${PROJECT_NAME}_node
        src/${PROJECT_NAME}.cpp
        src/${PROJECT_NAME}_node.cpp
        )

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${SENSEGLOVE}
    ${SENSEGLOVE_CONNECT})
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
