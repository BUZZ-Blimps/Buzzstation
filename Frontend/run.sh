#!/bin/bash

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
