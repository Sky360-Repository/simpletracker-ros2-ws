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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    camera_info_file = os.path.join(get_package_share_directory('simple_tracker_launch'), 'config', 'camera_info.yaml')
    config = os.path.join(get_package_share_directory('simple_tracker_launch'), 'config', 'params-simulation.yaml')

    #with open(config, 'r') as f:
    #    configuration = yaml.safe_load(f)
    #    print(f'Loaded configuration: {configuration}')

    return LaunchDescription([
        Node(
            package='simple_tracker_configuration',
            ##namespace='sky360',
            executable='configuration_service',
            parameters = [config],
            name='configuration_service',
        ),
        Node(
            package='simple_tracker_mask_provider',
            ##namespace='sky360',
            executable='mask_provider',
            name='mask_provider'
        ),
        Node(
            package='simple_tracker_annotated_frame_provider',
            #namespace='sky360',
            executable='annotated_frame_provider',
            name='annotated_frame_provider'
        ),
        Node(            
            package='simple_tracker_visualisers',
            #namespace='sky360',
            executable='simple_visualiser',
            remappings=[
                ('sky360/visualiser/annotated_frame', 'sky360/frames/annotated'),
            ],
            name='simple_visualiser',
        ),
    ])
