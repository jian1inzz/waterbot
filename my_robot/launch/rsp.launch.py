import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 宣告參數
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # robot_description 的 xacro 檔案位置
    xacro_path = PathJoinSubstitution([
        FindPackageShare('my_robot'),
        'description',
        'robot.urdf.xacro'
    ])

    robot_description_config = Command([
        'xacro ', xacro_path,
        ' use_sim:=', use_sim_time,
        ' use_ros2_control:=', use_ros2_control
    ])

    params = {
        'robot_description': robot_description_config,
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            name='use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        ),

        node_robot_state_publisher
    ])
