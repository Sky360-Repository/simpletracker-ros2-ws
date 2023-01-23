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
    
    #video_file = os.path.join(get_package_share_directory('simple_tracker_launch'), 'videos', 'plane_flying_past2.mkv')
    video_file = os.path.join(get_package_share_directory('simple_tracker_launch'), 'videos', 'brad_drone_1.mp4')
    #video_file = os.path.join(get_package_share_directory('simple_tracker_launch'), 'videos', 'Test_Trimmed.mp4')
    #video_file = os.path.join(get_package_share_directory('simple_tracker_launch'), 'videos', 'uap_texas_skyhub.mp4')
    #video_file = os.path.join(get_package_share_directory('simple_tracker_launch'), 'videos', 'Syn_ISS_Tracking.mp4')
    camera_info_file = os.path.join(get_package_share_directory('simple_tracker_launch'), 'config', 'camera_info.yaml')
    config = os.path.join(get_package_share_directory('simple_tracker_launch'), 'config', 'params.yaml')

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
        #Node(
        #    name='usb_cam',
        #    package='usb_cam',
        #    executable='usb_cam_node_exe',
        #    parameters = [config],
        #    remappings=[('image_raw', 'sky360/camera/original/v1')]
        #),
        Node(
            name='camera_simulator',
            package='camera_simulator',
            executable='camera_simulator',
            parameters = [config],
            remappings=[('/camera/image', 'sky360/camera/original')],
            arguments=[
                '--type', 'video', 
                '--path', video_file, 
                '--calibration_file', camera_info_file,
                '--loop']
        ),
        #Node(
        #    package='simple_tracker_camera',
        #    #namespace='sky360',
        #    executable='camera',
        #    name='camera'
        #),
        Node(
            package='simple_tracker_frame_provider',
            #namespace='sky360',
            executable='frame_provider',
            name='frame_provider'
        ),
        #Node(
        #    package='simple_tracker_dense_optical_flow_provider',
        #    #namespace='sky360',
        #    executable='dense_optical_flow_provider',
        #    name='dense_optical_flow_provider'
        #),
        Node(
            package='simple_tracker_background_subtraction_provider',
            #namespace='sky360',
            executable='background_subtraction_provider',
            name='background_subtraction_provider'
        ),
        Node(
            package='simple_tracker_detection_provider',
            #namespace='sky360',
            executable='bgs_detection_provider',
            name='detection_provider'
        ),
        Node(
            package='simple_tracker_track_provider',
            #namespace='sky360',
            executable='track_provider',
            name='track_provider'
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
            name='simple_visualiser',
            remappings=[
                ('sky360/visualiser/annotated_frame', 'sky360/frames/annotated'),
                #('sky360/visualiser/original_camera_frame', 'sky360/camera/original'),
                #('sky360/visualiser/original_frame', 'sky360/frames/original'),
                #('sky360/visualiser/masked_frame', 'sky360/frames/masked'),
                #('sky360/visualiser/grey_frame', 'sky360/frames/grey'),
                #('sky360/visualiser/dense_optical_flow_frame', 'sky360/frames/dense_optical_flow'),
                #('sky360/visualiser/masked_background_frame', 'sky360/frames/masked_background'),
                #('sky360/visualiser/foreground_mask_frame', 'sky360/frames/foreground_mask'),                
            ]
        ),
        #Node(
        #    package='rqt_image_view',
        #    executable='rqt_image_view',
        #    name='image_view',
        #    arguments=['image']
        #),
    ])
