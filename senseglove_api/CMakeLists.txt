# Copyright (c) 2020 - 2026 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_api)

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Find SenseGlove API libraries
set(SG_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/stage/lib/ros/x86-64/release)

find_library(SGCORE
        NAMES libsgcore.so sgcore
        PATHS ${SG_LIB_DIR}
        REQUIRED
        )

find_library(SGCONNECT
        NAMES libsgconnect.so sgconnect
        PATHS ${SG_LIB_DIR}
        REQUIRED
        )

find_library(SGSERIAL
        NAMES libserial.so serial
        PATHS ${SG_LIB_DIR}
        REQUIRED
        )

# Install headers
install(DIRECTORY stage/include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
        )

# Install libraries
install(FILES
  ${SGCORE}
  ${SGCONNECT}
  ${SGSERIAL}
  DESTINATION lib
)

add_executable(sg_tester src/sg_tester.cpp)

target_include_directories(sg_tester PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/stage/include
)

target_link_libraries(sg_tester
  ${SGCORE}
  ${SGCONNECT}
  ${SGSERIAL}
  pthread
)

ament_target_dependencies(sg_tester rclcpp)

install(TARGETS sg_tester
  DESTINATION lib/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${SGCORE} ${SGCONNECT} ${SGSERIAL})
ament_export_dependencies(rclcpp)

ament_package()