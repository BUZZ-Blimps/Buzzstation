#!/bin/bash

# Extract allowed SSIDs from Wifi_Networks.yaml using grep and sed
ALLOWED_SSIDS=$(grep -A 10 'allowed_wifi_networks:' Flask/src/Config/wifi_networks.yaml | sed -n "s/  - '\(.*\)'/\1/p")

# Get current SSID (Linux - nmcli)
SSID=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d':' -f2)

# Alternatively, for macOS, you can use:
# SSID=$(networksetup -getairportnetwork en0 | awk -F': ' '{print $2}')

if [ -z "$SSID" ]; then
  echo "Not connected to any Wi-Fi network."
  return 1
  exit 1
else
  echo "Connected to SSID: $SSID"
fi

# Check if SSID is in the allowed list
if echo "$ALLOWED_SSIDS" | grep -q "^$SSID$"; then
  echo "Connected to an allowed Wi-Fi network."
else
  echo "Not connected to an allowed Wi-Fi network. Exiting."
  return 1
  exit 1
fi

# Source ROS 2 Humble setup script
source /opt/ros/humble/setup.bash

# Check if /Backend/bin/activate exists
activate_script="/Backend/bin/activate"
if [ ! -f "$activate_script" ]; then
    echo "Error: Python virtual environment not set up yet. Execute setup.py using sudo."
    return 1
    exit 1
fi

# Source /Backend/bin/activate
source "$activate_script"

# ROS DOMAIN_ID
export ROS_DOMAIN_ID=1

# RWM IMPLEMENTATION
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Get the directory path of the script
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Get the directory of the run.sh script
run_dir="$script_dir/Flask/run"

# Execute run.sh script in run_dir
if [ -f "$run_dir/run.sh" ]; then
    echo "Executing run.sh in $run_dir"
    cd "$run_dir"  # Change directory to run_dir
    ./run.sh  # Execute run.sh script
else
    echo "Error: run.sh not found in $run_dir"
    return 1
    exit 1
fi
