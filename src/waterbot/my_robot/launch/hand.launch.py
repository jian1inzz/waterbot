from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='hand.py',
            name='hand',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='water2.py',
            name='water',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='enocder2.py',  # ✅ 這就是你這支超音波腳本
            name='xspeed',
            output='screen'
        )
    ])
