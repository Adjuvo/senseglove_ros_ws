#!/bin/sh

#
# Quick bash script to generate Docker images
#

usage() {
	echo "Usage ./build-image.sh [melodic|noetic]"
	exit 1
}

TAG=latest

if [ $# -ne 1 ]; then
	usage
fi

if [ $1 = "melodic" ] || [ $1 = "noetic" ]; then
	echo "Creating senseglove-$1-ros docker file"
	
	docker build $(echo $1 | tr 'm-n' 'M-N')/ -t senseglove-$1-ros:$TAG
	exit 0
else
	usage
fi

