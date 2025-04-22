import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_camera',
            output='screen',
            parameters=[{
                'image_size': [640, 480],
                'camera_frame_id': 'camera_link_optical'
            }],
            remappings=[
                ('/image_raw', '/usb_cam/image_raw'),
                ('/camera_info', '/usb_cam/camera_info')
            ]
        )
    ])