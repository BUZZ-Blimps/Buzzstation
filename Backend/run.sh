#!/bin/bash

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
