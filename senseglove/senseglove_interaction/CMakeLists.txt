# Copyright (c) 2020 - 2026 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_interaction
        VERSION 2.0.0
        DESCRIPTION "SenseGlove interaction nodes"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES NONE
        )

# Find dependencies
find_package(ament_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Export ament package
ament_package()