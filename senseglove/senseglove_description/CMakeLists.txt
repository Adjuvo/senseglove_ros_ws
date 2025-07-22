# Copyright (c) 2020 - 2025 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_description
        VERSION 2.0.0
        DESCRIPTION "The SenseGlove description package"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES C CXX
        )

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Include helper to build .xacro into .urdf
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(cmake/senseglove_description-extras.cmake)

# List of xacro inputs
set(XACRO_FILES
  urdf/dk1/dk1_left.xacro
  urdf/dk1/dk1_right.xacro
  urdf/nova1/nova_left.xacro
  urdf/nova1/nova_right.xacro
  urdf/nova2/nova2_left.xacro
  urdf/urdf_macros/transmissions.xacro
)

# Generate and Install the URDFs
build_and_install_xacro_files(${XACRO_FILES})

# Install raw xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.xacro"
)

# Install meshes
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install RViz configs
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Export ament package
ament_export_dependencies(xacro)
ament_package()