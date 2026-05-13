"""标定计算节点：加载采集数据，执行手眼标定。"""

import os
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from .utils.hand_eye_solver import solve_hand_eye, compute_reprojection_error
from .utils.transform_utils import (
    rvec_tvec_to_matrix, matrix_to_rvec_tvec, save_transform_yaml
)


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration')

        self.declare_parameter('arm_name', 'arm1')
        self.declare_parameter('calibration.method', 'PARK')
        self.declare_parameter('calibration.output_dir', 'calibration_data')
        self.declare_parameter('calibration.results_dir', 'results')

        self.arm_name = self.get_parameter('arm_name').value
        self.method = self.get_parameter('calibration.method').value
        self.data_dir = self.get_parameter('calibration.output_dir').value
        self.results_dir = self.get_parameter('calibration.results_dir').value

        os.makedirs(self.results_dir, exist_ok=True)

        self.calibrate_srv = self.create_service(
            Trigger, '~/run_calibration', self._calibrate_callback)

        self.get_logger().info(
            f'Calibration node ready for [{self.arm_name}]. '
            f'Call ~/run_calibration to compute.')

    def _calibrate_callback(self, request, response):
        """执行手眼标定计算。"""
        dataset_path = os.path.join(self.data_dir, self.arm_name, 'dataset.json')

        if not os.path.exists(dataset_path):
            response.success = False
            response.message = f'Dataset not found: {dataset_path}'
            return response

        with open(dataset_path, 'r') as f:
            dataset = json.load(f)

        samples = dataset['samples']
        if len(samples) < 3:
            response.success = False
            response.message = f'Need at least 3 samples, got {len(samples)}'
            return response

        # 准备数据
        R_gripper2base_list = []
        t_gripper2base_list = []
        R_target2cam_list = []
        t_target2cam_list = []

        for sample in samples:
            # 末端位姿 (base->ee)
            ee_mat = np.array(sample['ee_pose_matrix'])
            R_gripper2base_list.append(ee_mat[:3, :3])
            t_gripper2base_list.append(ee_mat[:3, 3].reshape(3, 1))

            # 标定板位姿 (board->camera)
            board_mat = rvec_tvec_to_matrix(
                sample['board_rvec'], sample['board_tvec'])
            R_target2cam_list.append(board_mat[:3, :3])
            t_target2cam_list.append(board_mat[:3, 3].reshape(3, 1))

        # 求解
        self.get_logger().info(
            f'Running hand-eye calibration with {len(samples)} samples, '
            f'method={self.method}')

        R_cam2ee, t_cam2ee = solve_hand_eye(
            R_gripper2base_list, t_gripper2base_list,
            R_target2cam_list, t_target2cam_list,
            method=self.method)

        # 计算误差
        rot_err, trans_err = compute_reprojection_error(
            R_gripper2base_list, t_gripper2base_list,
            R_target2cam_list, t_target2cam_list,
            R_cam2ee, t_cam2ee)

        # 构建4x4矩阵
        cam2ee_mat = np.eye(4)
        cam2ee_mat[:3, :3] = R_cam2ee
        cam2ee_mat[:3, 3] = t_cam2ee.flatten()

        # 保存结果
        result_path = os.path.join(
            self.results_dir, f'{self.arm_name}_hand_eye.yaml')
        save_transform_yaml(result_path, cam2ee_mat, name='cam_to_ee')

        msg = (
            f'Calibration complete for [{self.arm_name}].\n'
            f'  Rotation error: {rot_err:.4f} deg\n'
            f'  Translation error: {trans_err:.4f} mm\n'
            f'  Result saved: {result_path}')
        self.get_logger().info(msg)

        response.success = True
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
