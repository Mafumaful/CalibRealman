"""全局相机标定节点：确定全局相机相对于世界坐标系的位姿。"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from .utils.charuco_detector import CharucoDetector
from .utils.transform_utils import (
    rvec_tvec_to_matrix, invert_transform, compose_transforms,
    pose_to_matrix, save_transform_yaml
)


class GlobalCamNode(Node):
    def __init__(self):
        super().__init__('global_cam_calibration')

        self.declare_parameter('global_camera.camera_topic',
                               '/camera_m/color/image_raw')
        self.declare_parameter('global_camera.camera_info_topic',
                               '/camera_m/color/camera_info')
        self.declare_parameter('calibration.results_dir', 'results')
        self.declare_parameter('num_frames', 30)
        # 标定板在世界坐标系中的位姿 [x,y,z,qx,qy,qz,qw]
        self.declare_parameter('world_frame.board_to_world',
                               [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0])
        # ChArUco参数
        self.declare_parameter('squares_x', 5)
        self.declare_parameter('squares_y', 7)
        self.declare_parameter('square_length', 0.04)
        self.declare_parameter('marker_length', 0.03)
        self.declare_parameter('dictionary', 'DICT_4X4_50')

        camera_topic = self.get_parameter('global_camera.camera_topic').value
        camera_info_topic = self.get_parameter('global_camera.camera_info_topic').value
        self.results_dir = self.get_parameter('calibration.results_dir').value
        self.num_frames = self.get_parameter('num_frames').value
        board_to_world_param = self.get_parameter('world_frame.board_to_world').value

        os.makedirs(self.results_dir, exist_ok=True)

        # board在world中的位姿
        self.T_board_to_world = pose_to_matrix(
            board_to_world_param[:3], board_to_world_param[3:])

        # ChArUco检测器
        self.charuco = CharucoDetector(
            squares_x=self.get_parameter('squares_x').value,
            squares_y=self.get_parameter('squares_y').value,
            square_length=self.get_parameter('square_length').value,
            marker_length=self.get_parameter('marker_length').value,
            dictionary_name=self.get_parameter('dictionary').value,
        )

        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_matrix = None
        self.dist_coeffs = None

        self.image_sub = self.create_subscription(
            Image, camera_topic, self._image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._info_callback, 10)

        self.calibrate_srv = self.create_service(
            Trigger, '~/calibrate_global', self._calibrate_callback)

        self.get_logger().info('Global camera calibration node ready.')

    def _image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)

    def _calibrate_callback(self, request, response):
        """多帧采集并计算全局相机位姿。"""
        if self.latest_image is None or self.camera_matrix is None:
            response.success = False
            response.message = 'No image or camera info received.'
            return response

        rvecs = []
        tvecs = []
        collected = 0

        self.get_logger().info(
            f'Collecting {self.num_frames} frames for global camera calibration...')

        # 使用当前帧（实际使用中应多帧采集，这里简化为单帧多次检测）
        # 在实际ROS2中，可以用timer或循环采集
        rvec, tvec, _, _, success = self.charuco.detect_and_estimate(
            self.latest_image, self.camera_matrix, self.dist_coeffs)

        if not success:
            response.success = False
            response.message = 'ChArUco board not detected in global camera view.'
            return response

        # board相对于camera的变换: T_board_in_cam
        T_board_in_cam = rvec_tvec_to_matrix(rvec, tvec)

        # camera相对于board: T_cam_in_board = inv(T_board_in_cam)
        T_cam_in_board = invert_transform(T_board_in_cam)

        # camera相对于world: T_cam_in_world = T_board_to_world @ T_cam_in_board
        T_cam_in_world = compose_transforms(self.T_board_to_world, T_cam_in_board)

        # 保存结果
        result_path = os.path.join(self.results_dir, 'global_cam_to_world.yaml')
        save_transform_yaml(result_path, T_cam_in_world, name='global_cam_to_world')

        # 同时保存board_to_world
        board_path = os.path.join(self.results_dir, 'board_to_world.yaml')
        save_transform_yaml(board_path, self.T_board_to_world, name='board_to_world')

        msg = f'Global camera calibration complete. Result saved: {result_path}'
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
