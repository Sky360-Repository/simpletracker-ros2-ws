#!/bin/bash
set -e

# Add the ROS2 apt repository to your system.
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Update your apt repository caches after setting up the repositories.
sudo apt update
#sudo apt upgrade

#vcs import < src/ros2.repos src
#sudo apt-get update
rosdep update
#rosdep install --from-paths src --ignore-src -y
rosdep install -i --from-path src --rosdistro humble -y

# build packages
colcon build
# source build
source install/setup.bash

# run launch file
ros2 launch simple_tracker_launch simple_tracker_launch.py