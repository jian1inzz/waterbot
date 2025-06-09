import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_robot'

    # 設定 Gazebo 參數檔案路徑
    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config', 'gazebo_params.yaml'
    )

    # 設定 Gazebo plugin 使用的參數檔路徑（controller config）
    gazebo_controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config', 'mecanum_controller.yaml'
    )

    # 宣告 xacro 用到的 controller_config_path
    controller_config_arg = DeclareLaunchArgument(
        name='controller_config_path',
        default_value=gazebo_controller_params_file,
        description='Path to mecanum_controller.yaml for gazebo_ros2_control'
    )

    # 宣告 use_ros2_control
    use_ros2_control_arg = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='true',
        description='Use ros2_control if true'
    )

    # robot_state_publisher，讓 robot_description 有傳遞 xacro argument
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': LaunchConfiguration('use_ros2_control'),
            'controller_config_path': LaunchConfiguration('controller_config_path')
        }.items()
    )

    # 啟動 Gazebo 並傳入 gazebo_params.yaml
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': f'--ros-args --params-file {gazebo_params_file}'
        }.items()
    )

    # 把機器人實體送進 Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mecanum_bot'],
        output='screen'
    )

    # 自動啟動 joint_state_broadcaster 和 mecanum_controller
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller'],
        output='screen'
    )

    return LaunchDescription([
        controller_config_arg,
        use_ros2_control_arg,
        rsp,
        gazebo,
        spawn_entity,
        joint_broad_spawner,
        mecanum_controller_spawner
    ])