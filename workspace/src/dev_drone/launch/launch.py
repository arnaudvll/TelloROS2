from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dev_drone',
            executable='tello_behavior',
            name='tello_behavior'
        ),
        Node(
            package='dev_drone',
            executable='control',
            name='control',
            remappings=[
                ('/control', '/secure_cmd'),
                ('/flip', '/secure_flip')
            ],
        ),
        Node(
            package='rqt_image_view',
            namespace='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
        ),
        Node(
            package='tello',
            executable='tello',
            name='tello',
            remappings=[
                ('/image_raw', '/image'),
            ],
            
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='barcode_reader'
        )
    ])