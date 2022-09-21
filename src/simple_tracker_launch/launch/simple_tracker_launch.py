from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_tracker_configuration',
            ##namespace='sky360',
            executable='configuration_service',
            name='configuration_service'
        ),
        Node(
            package='simple_tracker_mask_provider',
            ##namespace='sky360',
            executable='mask_provider',
            name='mask_provider'
        ),        
        #Node(
        #    package='simple_tracker_camera',
        #    #namespace='sky360',
        #    executable='camera',
        #    name='camera'
        #),
        Node(
            package='simple_tracker_video',
            #namespace='sky360',
            executable='video',
            name='video'
        ),        
        Node(
            package='simple_tracker_frame_provider',
            #namespace='sky360',
            executable='frame_provider',
            name='frame_provider'
        ),
        Node(
            package='simple_tracker_dense_optical_flow_provider',
            #namespace='sky360',
            executable='dense_optical_flow_provider',
            name='dense_optical_flow_provider'
        ),
        Node(
            package='simple_tracker_background_subtraction_provider',
            #namespace='sky360',
            executable='foreground_mask_provider',
            name='foreground_mask_provider'
        ),
        Node(
            package='simple_tracker_detection_provider',
            #namespace='sky360',
            executable='detection_provider',
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
            name='simple_visualiser'
        ),        
    ])