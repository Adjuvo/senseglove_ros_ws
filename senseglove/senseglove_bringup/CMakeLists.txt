# Copyright (c) 2020 - 2026 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_bringup
        VERSION 2.0.0
        DESCRIPTION "Launch files for SenseGlove"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES NONE
        )

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/glove_connect.sh
    scripts/glove_disconnect.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()