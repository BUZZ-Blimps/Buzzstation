#!/bin/bash

export UNIQUE_ID=buzzstation

source /opt/ros/humble/setup.bash

ros2 launch basestation.launch.py
