"""发布标定结果TF的Launch文件。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('calib_realman')
    calib_config = os.path.join(pkg_dir, 'config', 'calibration_params.yaml')

    return LaunchDescription([
        Node(
            package='calib_realman',
            executable='tf_publisher',
            name='calibration_tf_publisher',
            parameters=[calib_config],
            output='screen',
        ),
    ])
