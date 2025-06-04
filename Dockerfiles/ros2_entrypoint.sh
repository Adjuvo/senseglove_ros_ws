#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/ros/jazzy/setup.bash

[ -f /ros-jazzy-ros1-bridge/install/local_setup.bash ] && \
  source /ros-jazzy-ros1-bridge/install/local_setup.bash

exec "$@"
