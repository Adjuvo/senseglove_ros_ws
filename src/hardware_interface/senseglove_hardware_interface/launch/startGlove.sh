#!/bin/bash

# Start SenseCom
$1/../SenseGlove_API/SenseCom/SenseCom.x86_64 &

# Allow SenseComm to start
sleep 3

# Start SenseGlove node
source ~/.bashrc
roslaunch senseglove_hardware_interface hardware.launch robot:=dk1 nr_of_glove:=1 is_right:=false
roslaunch senseglove_hardware_interface hardware.launch robot:=dk1 nr_of_glove:=2 is_right:=true

# Kill SenseComm when node is killed
killall -9 SenseCom.x86_64
