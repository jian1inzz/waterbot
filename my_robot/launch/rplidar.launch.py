import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                'serial_baudrate': 115200,  # A1M8 預設為 115200
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
    ])
