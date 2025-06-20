# Copyright (c) 2020 - 2025 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_shared_resources
        VERSION 3.0.0
        DESCRIPTION "The senseglove_shared_resources package for custom messages & services."
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES CXX
        )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)rovides
find_package(builtin_interfaces REQUIRED)

# Generate Messages & Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FingerDistanceFloats.msg"
  "msg/KinematicsVect3D.msg"
  "msg/SenseGloveState.msg"
  "srv/Calibrate.srv"
  DEPENDENCIES 
    std_msgs 
    geometry_msgs 
    trajectory_msgs 
    control_msgs 
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export ament package
ament_package()