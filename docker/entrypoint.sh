#!/bin/bash
set -e

: "${ROS_DISTRO:=jazzy}"
: "${ROS_WORKSPACE:=/root/workspace}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "${ROS_WORKSPACE}"

if [[ -f "${ROS_WORKSPACE}/install/setup.bash" ]]; then
  source "${ROS_WORKSPACE}/install/setup.bash"
else
  colcon build --symlink-install
  source "${ROS_WORKSPACE}/install/setup.bash"
  echo "SenseGlove container ready"
fi

exec "$@"