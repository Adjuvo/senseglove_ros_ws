#!/bin/bash

# Example script to run a container
# Please adjust accordingly
SENSEGLOVE_DEV0=/dev/tty0
SENSEGLOVE_DEV1=/dev/ttyACM1

WORKSPACE_DIR=~/senseglove-ros
DOCKER_IMAGE=senseglove-melodic-ros:latest

docker run --rm -it --privileged --net=host \
	--device=$SENSEGLOVE_DEV0 \
	--device=$SENSEGLOVE_DEV1 \
	--env="DISPLAY" \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--gpus all \
	-v $WORKSPACE_DIR:/project -w /project $DOCKER_IMAGE
