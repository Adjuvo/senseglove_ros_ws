#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/ros/jazzy/setup.bash

if [ -f /ros-jazzy-ros1-bridge/install/local_setup.bash ]; then
  source /ros-jazzy-ros1-bridge/install/local_setup.bash
fi

exec "$@"
