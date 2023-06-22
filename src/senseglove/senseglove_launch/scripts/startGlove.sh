#!/bin/bash

# Start SenseCom
chmod +x $1/SenseCom-1.3.1/Linux/SenseCom_Linux_Latest/SenseCom.x86_64
$1/SenseCom-1.3.1/Linux/SenseCom_Linux_Latest/SenseCom.x86_64 &

# Allow SenseComm to start
sleep 15

# Start SenseGlove node
source ~/.bashrc
roslaunch senseglove_launch senseglove_hardware_demo.launch left:=$2 right:=$3

# Kill SenseComm when node is killed
killall -9 SenseCom.x86_64
