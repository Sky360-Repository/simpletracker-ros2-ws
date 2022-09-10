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
            package='simple_tracker',
            #namespace='sky360',
            executable='frame_publisher',
            name='frame_publisher'
        ),
        Node(
            package='simple_tracker_visualisers',
            #namespace='sky360',
            executable='resized_frame_visualiser',
            name='resized_frame_visualiser'
        ),
        Node(
            package='simple_tracker_visualisers',
            #namespace='sky360',
            executable='full_frame_visualiser',
            name='full_frame_visualiser'
        ),        
    ])