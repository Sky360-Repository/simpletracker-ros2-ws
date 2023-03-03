# Original work Copyright (c) 2022 Sky360
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    launch_package_dir = get_package_share_directory('simple_tracker_launch')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/simple_tracker_launch.py'])
        ),

        Node(
            package='simple_tracker_monitor',
            #namespace='sky360',
            executable='prometheus_metrics',
            name='prometheus_metrics'
        ),
    ])
