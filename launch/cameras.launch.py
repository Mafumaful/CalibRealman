"""启动3个RealSense D435相机节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm1_serial', default_value='',
                             description='RealSense serial number for arm1 camera'),
        DeclareLaunchArgument('arm2_serial', default_value='',
                             description='RealSense serial number for arm2 camera'),
        DeclareLaunchArgument('global_serial', default_value='',
                             description='RealSense serial number for global camera'),

        # Arm1 camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='arm1',
            parameters=[{
                'serial_no': LaunchConfiguration('arm1_serial'),
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30.0,
                'enable_depth': False,
            }],
        ),

        # Arm2 camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='arm2',
            parameters=[{
                'serial_no': LaunchConfiguration('arm2_serial'),
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30.0,
                'enable_depth': False,
            }],
        ),

        # Global camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='global_camera',
            parameters=[{
                'serial_no': LaunchConfiguration('global_serial'),
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30.0,
                'enable_depth': False,
            }],
        ),
    ])
