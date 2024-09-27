#!/bin/bash

# Extract allowed SSIDs from Wifi_Networks.yaml using grep and sed
ALLOWED_SSIDS=$(grep -A 10 'allowed_wifi_networks:' ../Backend/Flask/src/Config/wifi_networks.yaml | sed -n "s/  - '\(.*\)'/\1/p")

# Get current SSID (Linux - nmcli)
SSID=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d':' -f2)

# Alternatively, for macOS, you can use:
# SSID=$(networksetup -getairportnetwork en0 | awk -F': ' '{print $2}')

if [ -z "$SSID" ]; then
  echo "Not connected to any Wi-Fi network."
  exit 1
else
  echo "Connected to SSID: $SSID"
fi

# Check if SSID is in the allowed list
if echo "$ALLOWED_SSIDS" | grep -q "^$SSID$"; then
  echo "Connected to an allowed Wi-Fi network."
else
  echo "Connected to a disallowed Wi-Fi network. Exiting."
  exit 1
fi

# Get the directory path of the script
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Get the directory of the run.sh script
run_dir="$script_dir/Expo/run"

# Execute run.sh script in run_dir
if [ -f "$run_dir/run.sh" ]; then
    echo "Executing run.sh in $run_dir"
    cd "$run_dir"  # Change directory to run_dir
    ./run.sh  # Execute run.sh script
else
    echo "Error: run.sh not found in $run_dir"
    exit 1
fi
