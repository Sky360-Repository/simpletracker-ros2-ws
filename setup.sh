#!/bin/bash
set -e

vcs import < src/ros2.repos src
sudo apt-get update

sudo apt install -y python3-natsort  # Installing natsort (is not in rosdep)

rosdep update
rosdep install --from-paths src --ignore-src -y

# cd /opt/Fast-DDS-statistics-backend/examples/cpp/ROS2Prometheus/build
# cmake .. && make


cd /workspaces/simpletracker-ros2-ws/

sudo cp rosbridge_websocket.launch /opt/ros/humble/share/rosbridge_server