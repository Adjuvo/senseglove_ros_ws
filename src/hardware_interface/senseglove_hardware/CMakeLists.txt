cmake_minimum_required(VERSION 3.0.2)
project(senseglove_hardware)

add_compile_options(-std=c++14 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        urdf
        )

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_library(SGCORE
        NAMES libSGCoreCpp.so
        PATHS ../../SenseGlove_API/Core/SGCoreCpp/lib/linux/Release
        REQUIRED)

find_library(SGCONNECT
        NAMES libSGConnect.so
        PATHS ../../SenseGlove_API/Core/SGConnect/lib/linux
        REQUIRED)

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
        ../../SenseGlove_API/Core/SGConnect/incl
        ../../SenseGlove_API/Core/SGCoreCpp/incl
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

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SGCONNECT} ${SGCORE} pthread)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(DIRECTORY SenseCom
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

# install(PROGRAMS SenseCom/Linux/SenseCom.x86_64 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

### Add gtest based cpp test target and link libraries
#if(CATKIN_ENABLE_TESTING)
#    catkin_add_gmock(${PROJECT_NAME}_test
#            test/.cpp
#            )
#    target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME})
#
#    if(ENABLE_COVERAGE_TESTING)
#        set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*" "*/${PROJECT_NAME}/check/*")
#        add_code_coverage(
#                NAME coverage_report
#                DEPENDENCIES ${PROJECT_NAME}_test
#        )
#    endif()
#endif()
