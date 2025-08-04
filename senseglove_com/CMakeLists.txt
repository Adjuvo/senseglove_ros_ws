# Copyright (c) 2020 - 2025 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_com
        VERSION 2.0.0
        DESCRIPTION "SenseGlove communication library"
        HOMEPAGE_URL "https://senseglove.com"
        )

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install
install(
  DIRECTORY SenseCom/Linux/SenseCom_Linux_Latest/
  DESTINATION share/${PROJECT_NAME}/Linux/SenseCom_Linux_Latest
)

install(DIRECTORY SenseCom/Linux/SenseCom_Linux_Latest/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
