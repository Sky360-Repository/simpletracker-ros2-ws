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
            package='camera_mock',
            #namespace='sky360',
            executable='camera',
            name='camera'
        ),
        Node(
            package='simple_tracker_frame_provider',
            #namespace='sky360',
            executable='frame_provider',
            name='frame_provider'
        ),        
        Node(
            package='simple_tracker_visualisers',
            #namespace='sky360',
            executable='simple_visualiser',
            name='simple_visualiser'
        ),        
    ])