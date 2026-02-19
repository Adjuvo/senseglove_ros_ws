# Copyright (c) 2020 - 2026 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)
project(senseglove_state_broadcaster
        VERSION 2.0.0
        DESCRIPTION "Publishes SenseGloveState from hardware state interfaces"
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
find_package(senseglove_msgs REQUIRED)

# Add library target
add_library(${PROJECT_NAME} SHARED 
        src/senseglove_state_broadcaster.cpp
        )

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        controller_interface
        pluginlib
        senseglove_msgs
        )

# Plugin registration
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

# Install headers
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp"
        )

# Install
install(FILES plugin.xml
        DESTINATION share/${PROJECT_NAME}
        )

# Install plugin
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Export ament package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface controller_interface pluginlib senseglove_msgs)

ament_package()