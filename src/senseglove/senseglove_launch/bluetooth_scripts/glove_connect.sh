#!/usr/bin/env bash

# Define color codes
BLUE='\e[34m'
GREEN='\e[32m'
BOLD='\e[1m'
RED='\e[31m'
RESET='\e[0m'

# Set up the Bluetooth agent to handle PIN confirmation
echo -e "agent DisplayYesNo\ndefault-agent" | sudo bluetoothctl

# Function to connect a Bluetooth device
connect_device() {
    local device="$1"
    local rfcomm="$2"
    
    if [[ -z "$device" ]]; then
        echo -e "${RED} No device selected! Skipping connection.${RESET}"
        return
    fi
    
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
echo "Step 1: Listing discovered NOVA 2 gloves..."
mapfile -t devices < <(bluetoothctl devices | grep -i "nova" | awk '{print $2}')
mapfile -t names < <(bluetoothctl devices | grep -i "nova" | awk '{for(i=3;i<=NF;i++) printf $i" "; print ""}')

# Check if any gloves were found
if [[ ${#devices[@]} -eq 0 ]]; then
    echo -e "${RED} No Nova gloves detected! Check Bluetooth connections.${RESET}"
    exit 1
fi

# Display available gloves
echo "Found Nova gloves:"
for i in "${!devices[@]}"; do
    echo "  [$((i+1))] ${names[i]}"
done

# Step 2: User selects the first glove
read -p "Step 2: Enter the number of the first glove you want to connect: " choice
if [[ -z "$choice" || "$choice" -lt 1 || "$choice" -gt "${#devices[@]}" ]]; then
    echo -e "${RED} Invalid selection. Exiting.${RESET}"
    exit 1
fi
SG_DEVICE0="${devices[$((choice-1))]}"
SG_NAME0="${names[$((choice-1))]}"

# Step 3: User selects the second glove (optional)
read -p "Step 3: Enter the number of the second glove to connect (or press Enter to skip): " choice
if [[ -n "$choice" && "$choice" -ge 1 && "$choice" -le "${#devices[@]}" ]]; then
    SG_DEVICE1="${devices[$((choice-1))]}"
    SG_NAME1="${names[$((choice-1))]}"
else
    SG_DEVICE1=""
    SG_NAME1=""
fi

# Ensure that at least one device is selected
if [[ -z "$SG_DEVICE0" ]]; then
    echo -e "${RED} No valid selection made. Exiting.${RESET}"
    exit 1
fi

# Set the correct RFCOMM assignments
SG_RFCOMM0="/dev/rfcomm0"
SG_RFCOMM1="/dev/rfcomm1"

# Automatically detect left and right from names using shell pattern matching
SG_LEFT=""
SG_RIGHT=""

if [[ "$SG_NAME0" == *"-L"* ]]; then
    SG_LEFT="$SG_DEVICE0"
elif [[ "$SG_NAME0" == *"-R"* ]]; then
    SG_RIGHT="$SG_DEVICE0"
fi

if [[ -n "$SG_DEVICE1" ]]; then
    if [[ "$SG_NAME1" == *"-L"* ]]; then
        SG_LEFT="$SG_DEVICE1"
    elif [[ "$SG_NAME1" == *"-R"* ]]; then
        SG_RIGHT="$SG_DEVICE1"
    fi
fi

# Connect gloves with proper assignment
if [[ -n "$SG_LEFT" && -n "$SG_RIGHT" ]]; then
    echo -e "${GREEN}Connecting LEFT glove ($SG_LEFT) to $SG_RFCOMM0...${RESET}"
    connect_device "$SG_LEFT" "$SG_RFCOMM0"

    echo -e "${GREEN}Connecting RIGHT glove ($SG_RIGHT) to $SG_RFCOMM1...${RESET}"
    connect_device "$SG_RIGHT" "$SG_RFCOMM1"

else
    # If only one glove is selected, it should always connect to rfcomm0
    SINGLE_GLOVE="${SG_LEFT:-$SG_RIGHT}"
    
    if [[ -z "$SINGLE_GLOVE" ]]; then
        echo -e "${RED} No valid glove detected! Exiting.${RESET}"
        exit 1
    fi
    
    echo -e "${GREEN}Only one glove selected ($SINGLE_GLOVE). Connecting it to $SG_RFCOMM0...${RESET}"
    connect_device "$SINGLE_GLOVE" "$SG_RFCOMM0"
fi

echo -e "${GREEN}${BOLD}All connection processes have completed.${RESET}"
