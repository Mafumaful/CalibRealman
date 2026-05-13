from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'calib_realman'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Dual-arm hand-eye calibration for Realman RM65 with RealSense D435',
    license='MIT',
    entry_points={
        'console_scripts': [
            'data_collector = calib_realman.data_collector_node:main',
            'calibration = calib_realman.calibration_node:main',
            'global_cam = calib_realman.global_cam_node:main',
            'tf_publisher = calib_realman.tf_publisher_node:main',
            'arm_driver = calib_realman.arm_driver_node:main',
            'diagnose_charuco = calib_realman.diagnose_charuco_node:main',
            'keyboard_trigger = scripts.keyboard_trigger:main',
        ],
    },
)
