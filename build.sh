#!/bin/bash
set -e

# build packages
colcon build
# source build
source install/setup.bash

# run launch file
ros2 launch simple_tracker_launch simple_tracker_launch.py
