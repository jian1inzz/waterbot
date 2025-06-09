from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='camera.py',
            name='camera_ws_node',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='yolorun.py',
            name='yolo_cmd_listener',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='hcsr04.py',  # ✅ 這就是你這支超音波腳本
            name='ultrasonic_sensor_node',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='encoder.py',  # ✅ 這就是你這支超音波腳本
            name='xspeed',
            output='screen'
        )
    ])
