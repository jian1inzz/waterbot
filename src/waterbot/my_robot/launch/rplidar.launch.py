from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 假設 rplidar_ros 是 ROS 2 package，使用 find_package 來定位路徑
    from ament_index_python.packages import get_package_share_directory

    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'), 
        'launch', 
        'rplidar_a1_launch.py'  # 這是你的雷達啟動檔名
    )

    return LaunchDescription([
        # 啟動 RPLIDAR 節點
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path)
        ),
        # 啟動 Python 篩選節點
        Node(
            package='my_robot',  # 這裡換成你的 package 名稱
            executable='rplidar.py',  #
            name='lidar_filter_node',
            output='screen'
        )
    ])
