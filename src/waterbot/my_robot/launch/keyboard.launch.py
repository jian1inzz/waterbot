from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',   # ⬅️ 請改成你實際的 package 名稱
            executable='keyboard.py', # ⬅️ 你的 Python 檔名去掉 `.py`
            name='keyboard_node',
            output='screen',
            emulate_tty=True  # 這行可讓 print 顯示在畫面上
        )
    ])