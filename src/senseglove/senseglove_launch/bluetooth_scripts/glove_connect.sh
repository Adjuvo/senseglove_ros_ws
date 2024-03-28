#!/usr/bin/bash

function connect_device() {
    local device=$1
    local rfcomm=$2

    sudo bluetoothctl pair "$device"
    sudo bluetoothctl trust "$device"
    sudo bluetoothctl connect "$device"
    sudo rfcomm connect "$rfcomm" "$device" 1 &
}

# Step 1: List Bluetooth devices and prompt for the correct one
echo "Step 1: Listing Bluetooth devices..."
bluetoothctl devices | grep -i "nova"

# Prompt the user to copy the correct MAC address
read -p "Step 2: Enter the MAC address of a NOVA glove (copy from the list): " SG_DEVICE0

# Prompt the user to copy the correct MAC address
read -p "Step 3: Enter the MAC address of another NOVA glove. If not necessary, press enter: " SG_DEVICE1

# Set your device and rfcomm variables
SG_RFCOMM0="/dev/rfcomm0"
SG_RFCOMM1="/dev/rfcomm1"

# Call the function for each device
connect_device $SG_DEVICE0 $SG_RFCOMM0

# Call the function for SG_DEVICE1 only if it's not empty
if [ -n "$SG_DEVICE1" ]; then
    connect_device "$SG_DEVICE1" "$SG_RFCOMM1"
fi
connect_device $SG_DEVICE1 $SG_RFCOMM1

