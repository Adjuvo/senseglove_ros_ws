#!/usr/bin/bash

# Define color codes
BLUE='\e[34m'
GREEN='\e[32m'
BOLD='\e[1m'

# Function to disconnect and release a device
disconnect_and_release() {
    local rfcomm=$1

    output=$(rfcomm show $rfcomm)
    device=$(echo "$output" | awk -F' ' '{print $4}')

    bluetoothctl disconnect ${device} 
    rfcomm release ${rfcomm}
}

# Set your rfcomm variables
SG_RFCOMM0="/dev/rfcomm0"
SG_RFCOMM1="/dev/rfcomm1"

echo -e "${BLUE}==> Disconnecting and releasing all paired devices ...${RESET}"

# Call the function for each device
disconnect_and_release $SG_RFCOMM0
disconnect_and_release $SG_RFCOMM1

echo -e "${GREEN}${BOLD} All devices disconnected!${RESET}"