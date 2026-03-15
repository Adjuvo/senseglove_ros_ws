# Copyright (c) 2020 - 2026 SenseGlove

cmake_minimum_required(VERSION 3.21...3.25)

project(senseglove_msgs
        VERSION 2.0.0
        DESCRIPTION "Custom msg types for SenseGlove"
        HOMEPAGE_URL "https://senseglove.com"
        LANGUAGES C CXX
        )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate Messages & Services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FingerDistanceFloats.msg"
  "msg/KinematicsVect3D.msg"
  "msg/Nova2WaveformCommand.msg"
  "msg/SenseGloveState.msg"
  "srv/Calibrate.srv"
  DEPENDENCIES std_msgs geometry_msgs control_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export ament package
ament_package()