#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/ros/jazzy/setup.bash

source /senseglove_shared_resources_msgs/ros1/install/local_setup.bash
source /senseglove_shared_resources_msgs/ros2/install/local_setup.bash

if [ -f /ros-jazzy-ros1-bridge/install/local_setup.bash ]; then
  source /ros-jazzy-ros1-bridge/install/local_setup.bash
fi

exec "$@"
