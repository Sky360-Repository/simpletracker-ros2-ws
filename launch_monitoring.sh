#!/bin/bash
set -e

# MWG: Look into moving this into VSCode tasks versus this launch script, 
# source build
source install/setup.bash

# run launch file
ROS2Prometheus & ros2 launch simple_tracker_launch simple_tracker_launch.py && kill $!