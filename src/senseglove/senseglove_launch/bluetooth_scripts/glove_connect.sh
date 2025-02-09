#!/usr/bin/env bash

# Define color codes
BLUE='\e[34m'
GREEN='\e[32m'
BOLD='\e[1m'

# Set up the Bluetooth agent to handle PIN confirmation
echo -e "agent DisplayYesNo\ndefault-agent" | sudo bluetoothctl

# Function to connect a Bluetooth device (no retries)
connect_device() {
    local device="$1"
    local rfcomm="$2"

    echo -e "${BLUE}==> Pairing device $device ...${RESET}"
    sudo bluetoothctl pair "$device"
    echo -e "${BLUE}==> Trusting device $device ...${RESET}"
    sudo bluetoothctl trust "$device"
    echo -e "${BLUE}==> Connecting to device $device ...${RESET}"
    sudo bluetoothctl connect "$device"
    echo -e "${BLUE}==> Establishing RFCOMM connection on $rfcomm for device $device ...${RESET}"
    sudo rfcomm connect "$rfcomm" "$device" 1 &
}

# Scan for NOVA devices for a short period
echo "Scanning for all NOVA devices for 5 seconds..."
sudo bluetoothctl scan on > /dev/null 2>&1 &
sleep 5
sudo bluetoothctl scan off > /dev/null 2>&1

# Step 1: List discovered Bluetooth devices
echo "Step 1: Listing discovered Bluetooth devices..."
# Get devices that have "nova" (case insensitive) in their description
mapfile -t devices < <(bluetoothctl devices | grep -i "nova" | awk '{print $2}')
mapfile -t names < <(bluetoothctl devices | grep -i "nova" | awk '{for(i=3;i<=NF;i++) printf $i" "; print ""}')

echo "Found NOVA devices:"
for i in "${!devices[@]}"; do
    echo "  [$((i+1))] ${names[i]}"
done

# Prompt the user to choose the first device
read -p "Step 2: Enter the number corresponding to a NOVA glove: " choice
SG_DEVICE0="${devices[$((choice-1))]}"

# Prompt the user to choose the second device (optional)
read -p "Step 3: Enter the number corresponding to another NOVA glove (or press Enter to skip): " choice
if [[ -n "$choice" ]]; then
    SG_DEVICE1="${devices[$((choice-1))]}"
else
    SG_DEVICE1=""
fi

# Set your RFCOMM device variables
SG_RFCOMM0="/dev/rfcomm0"
SG_RFCOMM1="/dev/rfcomm1"

# Connect the first device
echo -e "${GREEN}Connecting first device ($SG_DEVICE0) on $SG_RFCOMM0...${RESET}"
connect_device "$SG_DEVICE0" "$SG_RFCOMM0"

# Connect the second device if provided
if [[ -n "$SG_DEVICE1" ]]; then
    echo -e "${GREEN}Connecting second device ($SG_DEVICE1) on $SG_RFCOMM1...${RESET}"
    connect_device "$SG_DEVICE1" "$SG_RFCOMM1"
fi

echo -e "${GREEN}${BOLD}All connection processes have completed.${RESET}"
