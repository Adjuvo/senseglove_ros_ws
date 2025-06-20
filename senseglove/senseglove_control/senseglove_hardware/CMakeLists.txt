# Copyright (c) 2020 - 2025 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_hardware
        VERSION 2.1.0
        DESCRIPTION "The SenseGlove Hardware for communication with the Device"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Werror)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED) 

# Find SenseGlove API libraries
find_library(SGCORE
        NAMES libsgcore.so sgcore
        PATHS ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/lib/linux/Ubuntu/x86-64/release/
        REQUIRED
        )

find_library(SGCONNECT
        NAMES libsgconnect.so sgconnect
        PATHS ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/lib/linux/Ubuntu/x86-64/release/
        REQUIRED
        )

find_library(SGSERIAL
        NAMES libserial.so serial
        PATHS ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/lib/linux/Ubuntu/x86-64/release/
        REQUIRED
        )

# Add library target
add_library(${PROJECT_NAME} SHARED
        src/joint.cpp
        src/senseglove_robot.cpp
        src/senseglove_setup.cpp
        )

# Include directoriesg
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # SenseGlove API includes
        ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/include/SenseGlove/Core
        ${CMAKE_CURRENT_LIST_DIR}/../../../senseglove_api/include/SenseGlove/Connect
        )

# Link against ROS 2 packages and SenseGlove libraries
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        urdf
        )
        
target_link_libraries(${PROJECT_NAME}
        ${SGCORE}
        ${SGCONNECT}
        ${SGSERIAL}
        pthread
        )

# Ensure RPATH is set so runtime linker can find shared libs if installed in non-standard location
set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH_USE_LINK_PATH TRUE
        )

# Install public headers
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )

# Install SenseCom folder
install(DIRECTORY SenseCom
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
ament_export_dependencies(rclcpp urdf)

ament_package()