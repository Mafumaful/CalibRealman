"""Launch: 相机位姿实时监视"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('calib_realman')
    cfg = os.path.join(pkg, 'config', 'calibration_params.yaml')

    arm_arg = DeclareLaunchArgument('arm_name', default_value='arm1',
                                    description='arm1 or arm2')

    node = Node(
        package='calib_realman',
        executable='cam_pose_monitor',
        name='cam_pose_monitor',
        parameters=[
            cfg,
            {
                # 从 arm_name 派生相机 topic（也可在 yaml 中覆盖）
                'camera_topic':
                    ['/camera_', LaunchConfiguration('arm_name'),
                     '/color/image_raw'],
                'camera_info_topic':
                    ['/camera_', LaunchConfiguration('arm_name'),
                     '/color/camera_info'],
            },
        ],
        output='screen',
    )

    return LaunchDescription([arm_arg, node])
