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
devices=($(bluetoothctl devices | grep -i "nova" | awk '{print $2}'))
names=($(bluetoothctl devices | grep -i "nova" | awk '{print $3 $4}'))

# Display devices with numbers
for i in "${!devices[@]}"; do
    echo "[$((i+1))] ${names[i]}"
done

# Prompt the user to choose a device
read -p "Step 2: Enter the number corresponding to a NOVA glove: " choice
SG_DEVICE0="${devices[choice-1]}"

# Prompt the user to choose a device
read -p "Step 3: Enter the number corresponding to another NOVA glove: " choice
SG_DEVICE1="${devices[choice-1]}"

# Set your device and rfcomm variables
SG_RFCOMM0="/dev/rfcomm0"
SG_RFCOMM1="/dev/rfcomm1"

# Call the function for each device
connect_device $SG_DEVICE0 $SG_RFCOMM0

# Call the function for SG_DEVICE1 only if it's not empty
if [ -n "$SG_DEVICE1" ]; then
    connect_device "$SG_DEVICE1" "$SG_RFCOMM1"

