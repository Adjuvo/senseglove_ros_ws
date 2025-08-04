# Copyright (c) 2020 - 2025 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_hardware_interface
        VERSION 2.0.0
        DESCRIPTION "The senseglove_hardware_interface for the bridge between ros_control and the senseglove hardware."
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
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(senseglove_hardware REQUIRED)
find_package(senseglove_hardware_builder REQUIRED)
find_package(senseglove_msgs REQUIRED)

# Find SenseGlove API libraries
find_library(SGCORE
        NAMES libsgcore.so sgcore
        PATHS ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/lib/Ubuntu/x86-64/release/
        REQUIRED
        )

find_library(SGCONNECT
        NAMES libsgconnect.so sgconnect
        PATHS ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/lib/Ubuntu/x86-64/release/
        REQUIRED
        )

find_library(SGSERIAL
        NAMES libserial.so serial
        PATHS ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/lib/Ubuntu/x86-64/release/
        REQUIRED
        )

# Add library target
add_library(${PROJECT_NAME} SHARED
        src/senseglove_hardware_interface.cpp
        )

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # SenseGlove API includes
        ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/include/SenseGlove/Core
        ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/include/SenseGlove/Connect
        ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/include/SenseGlove/BLE
        )

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        pluginlib
        senseglove_hardware
        senseglove_hardware_builder
        senseglove_msgs
        )

target_link_libraries(${PROJECT_NAME}
        ${SGCORE}
        ${SGCONNECT}
        ${SGSERIAL}
        )     

# Plugin registration
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

# Install headers
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
        )

# Install controllers yaml files
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
        )

install(FILES plugin.xml
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
ament_export_dependencies(hardware_interface pluginlib senseglove_hardware_builder senseglove_msgs)

ament_package()