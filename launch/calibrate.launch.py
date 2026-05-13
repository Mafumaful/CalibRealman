"""标定计算Launch文件。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('calib_realman')
    calib_config = os.path.join(pkg_dir, 'config', 'calibration_params.yaml')
    charuco_config = os.path.join(pkg_dir, 'config', 'charuco_board.yaml')

    return LaunchDescription([
        # Arm1 calibration
        Node(
            package='calib_realman',
            executable='calibration',
            name='calibration_arm1',
            parameters=[calib_config, {'arm_name': 'arm1'}],
            output='screen',
        ),

        # Arm2 calibration
        Node(
            package='calib_realman',
            executable='calibration',
            name='calibration_arm2',
            parameters=[calib_config, {'arm_name': 'arm2'}],
            output='screen',
        ),

        # Global camera calibration
        Node(
            package='calib_realman',
            executable='global_cam',
            name='global_cam_calibration',
            parameters=[calib_config, charuco_config],
            output='screen',
        ),
    ])
