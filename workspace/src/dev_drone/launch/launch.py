from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dev_drone',
            executable='control',
            name='control',
        ),
        Node(
            package='rqt_image_view',
            namespace='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
        )
    ])