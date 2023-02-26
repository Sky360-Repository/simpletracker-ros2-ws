#!/bin/bash
set -e

# MWG: Look into moving this into VSCode tasks versus this launch script, 
# source build
source install/setup.bash

export FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;NETWORK_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;SUBSCRIPTION_THROUGHPUT_TOPIC;RTPS_SENT_TOPIC;RTPS_LOST_TOPIC;HEARTBEAT_COUNT_TOPIC;ACKNACK_COUNT_TOPIC;NACKFRAG_COUNT_TOPIC;GAP_COUNT_TOPIC;DATA_COUNT_TOPIC;RESENT_DATAS_TOPIC;SAMPLE_DATAS_TOPIC;PDP_PACKETS_TOPIC;EDP_PACKETS_TOPIC;DISCOVERY_TOPIC;PHYSICAL_DATA_TOPIC"

# run launch file
#ROS2Prometheus & ros2 launch simple_tracker_launch rosbridge_websocket.launch.xml & ros2 launch simple_tracker_launch monitor_launch.py && kill $!

ros2 launch simple_tracker_launch monitor_launch.py

#ROS2Prometheus
#ros2 launch simple_tracker_launch rosbridge_websocket.launch.xml
#ros2 launch simple_tracker_launch monitor_launch.py