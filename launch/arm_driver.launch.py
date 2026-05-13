"""启动睿尔曼机械臂驱动节点，发布 base->ee TF。

用法:
    # 启动单臂
    ros2 launch calib_realman arm_driver.launch.py arm_name:=arm1
    ros2 launch calib_realman arm_driver.launch.py arm_name:=arm2

如需同时启动两个臂，请在两个终端分别启动。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('calib_realman')
    calib_config = os.path.join(pkg_dir, 'config', 'calibration_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('arm_name', default_value='arm1',
                             description='arm1 or arm2'),

        Node(
            package='calib_realman',
            executable='arm_driver',
            name='arm_driver',
            parameters=[
                calib_config,
                {'arm_name': LaunchConfiguration('arm_name')},
            ],
            output='screen',
        ),
    ])
