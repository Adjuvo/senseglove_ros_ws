# Copyright (c) 2020 - 2026 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_hardware_builder
        VERSION 2.0.0
        DESCRIPTION "Builds senseglove hardware"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(senseglove_hardware REQUIRED)

# Find SenseGlove API libraries
set(SG_API_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api)
set(SG_LIB_DIR ${SG_API_DIR}/stage/lib/ros/x86-64/release)

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

# Add library target
add_library(${PROJECT_NAME} SHARED
        src/hardware_builder.cpp
        )

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # SenseGlove API includes
        ${SG_API_DIR}/stage/include
        ${SG_API_DIR}/stage/include/SenseGlove/Core
        ${SG_API_DIR}/stage/include/SenseGlove/Connect
        ${SG_API_DIR}/stage/include/SenseGlove/BLE
        )

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        ament_index_cpp
        urdf
        senseglove_hardware
        yaml-cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${SGCORE}
        ${SGCONNECT}
        ${SGSERIAL}
        pthread
        yaml-cpp
        )

# Ensure RPATH is set so runtime linker can find shared libs if installed in non-standard location
set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH_USE_LINK_PATH TRUE
        )

# Install headers
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
        )

# Install robot yaml files
install(DIRECTORY robots
        DESTINATION share/${PROJECT_NAME}
        )

# Install library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

# Export ament package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp urdf senseglove_hardware yaml-cpp)

ament_package()