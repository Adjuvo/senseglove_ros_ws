#!/usr/bin/bash

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

# Call the function for each device
disconnect_and_release $SG_RFCOMM0
disconnect_and_release $SG_RFCOMM1
