#!/bin/bash
set -e

# MWG: Look into moving this into VSCode tasks versus this launch script,
# source build
source install/setup.bash

# run launch file
ros2 launch simple_tracker_launch simple_tracker_qhy_launch.py &

# wait for ros2 to start
sleep 5

# launch rosbridge_server
ros2 launch rosbridge_websocket.launch

# wait for ros2 to start
sleep 10

ros2 run simple_tracker_single_frame_classifier single_frame_classifier
