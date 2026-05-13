"""数据采集Launch文件。通过arm_name参数选择采集哪个臂。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('calib_realman')
    charuco_config = os.path.join(pkg_dir, 'config', 'charuco_board.yaml')
    calib_config = os.path.join(pkg_dir, 'config', 'calibration_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('arm_name', default_value='arm1',
                             description='Which arm to collect data for (arm1/arm2)'),

        Node(
            package='calib_realman',
            executable='data_collector',
            name='data_collector',
            parameters=[
                calib_config,
                charuco_config,
                {'arm_name': LaunchConfiguration('arm_name')},
            ],
            output='screen',
        ),

        # 键盘触发节点
        ExecuteProcess(
            cmd=['ros2', 'run', 'calib_realman', 'keyboard_trigger'],
            output='screen',
        ),
    ])
