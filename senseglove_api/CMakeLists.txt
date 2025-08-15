# Copyright (c) 2020 - 2025 SenseGlove

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
find_library(SGCORE
        NAMES libsgcore.so sgcore
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/Ubuntu/x86-64/release
        REQUIRED
        )

find_library(SGCONNECT
        NAMES libsgconnect.so sgconnect
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/Ubuntu/x86-64/release
        REQUIRED
        )

find_library(SGSERIAL
        NAMES libserial.so serial
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/Ubuntu/x86-64/release
        REQUIRED
        )

# Install headers
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
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
  ${CMAKE_CURRENT_SOURCE_DIR}/include
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