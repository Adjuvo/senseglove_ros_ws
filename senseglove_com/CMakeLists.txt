# Copyright (c) 2020 - 2025 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_com
        VERSION 2.0.0
        DESCRIPTION "SenseGlove communication library"
        HOMEPAGE_URL "https://senseglove.com"
        )

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY SenseCom/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*"
)

ament_package()
