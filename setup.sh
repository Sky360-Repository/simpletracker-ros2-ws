#!/bin/bash
set -e

vcs import < src/ros2.repos src
sudo apt-get update

sudo apt install -y python3-natsort  # Installing natsort (is not in rosdep)

rosdep update
rosdep install --from-paths src --ignore-src -y