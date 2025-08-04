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

add_executable(check_sensecom src/check_sensecom.cpp)

target_include_directories(check_sensecom PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(check_sensecom
  ${SGCORE}
  ${SGCONNECT}
  ${SGSERIAL}
  pthread
)

ament_target_dependencies(check_sensecom rclcpp)

install(TARGETS check_sensecom
  DESTINATION lib/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${SGCORE} ${SGCONNECT} ${SGSERIAL})
ament_export_dependencies(rclcpp)

ament_package()