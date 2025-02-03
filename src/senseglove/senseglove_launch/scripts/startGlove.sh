#!/bin/bash

# Define color codes
BLUE='\e[34m'
GREEN='\e[32m'
BOLD='\e[1m'

# Start SenseCom
echo -e "${BLUE}Starting SenseCom...${RESET}"
chmod +x $1/SenseCom/Linux/SenseCom_Latest/SenseCom.x86_64
$1/SenseCom/Linux/SenseCom_Latest/SenseCom.x86_64 &

# Allow SenseComm to start
sleep 2

# Function to wait for user input
wait_for_user_input() {
    echo -e "${BOLD}${GREEN}Press enter when devices are connected in SenseCom...${RESET}"
    read
}

wait_for_user_input

# Start SenseGlove node
echo "Launching Senseglove"
source ~/.bashrc
roslaunch senseglove_launch senseglove_hardware.launch left:=$2 right:=$3 use_dk:=$4 use_nova:=$5 use_nova2:=$6

# Kill SenseComm when node is killed
killall -9 SenseCom.x86_64
