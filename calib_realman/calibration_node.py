"""标定计算节点：加载采集数据，执行手眼标定。"""

import os
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

from .utils.hand_eye_solver import solve_hand_eye, compute_reprojection_error
from .utils.transform_utils import (
    rvec_tvec_to_matrix, matrix_to_rvec_tvec, save_transform_yaml
)


# auto 模式尝试的约定列表（小写=外旋，大写=内旋）
AUTO_CONVENTIONS = ['xyz', 'zyx', 'XYZ', 'ZYX', 'xzy', 'yxz', 'yzx', 'zxy']


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration')

        self.declare_parameter('arm_name', 'arm1')
        self.declare_parameter('calibration.method', 'PARK')
        self.declare_parameter('calibration.output_dir', 'calibration_data')
        self.declare_parameter('calibration.results_dir', 'results')
        self.declare_parameter('calibration.min_corners', 0)
        # 若非空，从 raw_pose 用此约定重算 EE 位姿矩阵
        # 特殊值: 'auto' = 遍历所有常见约定，报告最佳
        # 其他: 'xyz'/'XYZ'/'zyx'/'ZYX' 等
        self.declare_parameter('calibration.euler_convention', '')

        self.arm_name = self.get_parameter('arm_name').value
        self.method = self.get_parameter('calibration.method').value
        self.data_dir = self.get_parameter('calibration.output_dir').value
        self.results_dir = self.get_parameter('calibration.results_dir').value
        self.min_corners = self.get_parameter('calibration.min_corners').value
        self.euler_conv = self.get_parameter('calibration.euler_convention').value

        os.makedirs(self.results_dir, exist_ok=True)

        self.calibrate_srv = self.create_service(
            Trigger, '~/run_calibration', self._calibrate_callback)

        self._print_loaded_params()

        self.get_logger().info(
            f'Calibration node ready for [{self.arm_name}]. '
            f'Call ~/run_calibration to compute.')

    def _print_loaded_params(self):
        """启动时打印所有加载的配置参数。"""
        conv_display = repr(self.euler_conv) if self.euler_conv else "(use ee_pose_matrix)"
        lines = [
            '=' * 60,
            f'  Calibration Node Loaded Parameters [{self.arm_name}]',
            '=' * 60,
            f'  arm_name         : {self.arm_name}',
            f'  method           : {self.method}',
            f'  data_dir         : {self.data_dir}',
            f'  results_dir      : {self.results_dir}',
            f'  min_corners      : {self.min_corners} (0=off)',
            f'  euler_convention : {conv_display}',
            '=' * 60,
        ]
        for line in lines:
            self.get_logger().info(line)

    def _load_samples(self):
        """加载并按min_corners过滤样本。返回 (samples, err_msg)。"""
        dataset_path = os.path.join(self.data_dir, self.arm_name, 'dataset.json')
        if not os.path.exists(dataset_path):
            return None, f'Dataset not found: {dataset_path}'

        with open(dataset_path, 'r') as f:
            dataset = json.load(f)
        samples = dataset['samples']
        if len(samples) < 3:
            return None, f'Need at least 3 samples, got {len(samples)}'

        if self.min_corners > 0:
            before = len(samples)
            samples = [s for s in samples
                       if s.get('corners_detected', 0) >= self.min_corners]
            self.get_logger().info(
                f'Filter min_corners={self.min_corners}: '
                f'{before} -> {len(samples)}')
            if len(samples) < 3:
                return None, f'After filter only {len(samples)} samples (<3).'

        return samples, None

    def _build_pose_lists(self, samples, euler_conv):
        """根据 euler_conv 构建手眼标定输入。
           euler_conv 为 None/'' 时用 ee_pose_matrix。"""
        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        skipped = 0
        for s in samples:
            if euler_conv:
                raw = s.get('raw_pose')
                if raw is None:
                    skipped += 1
                    continue
                x, y, z, rx, ry, rz = raw
                R = Rotation.from_euler(euler_conv, [rx, ry, rz]).as_matrix()
                ee_mat = np.eye(4)
                ee_mat[:3, :3] = R
                ee_mat[:3, 3] = [x, y, z]
            else:
                ee_mat = np.array(s['ee_pose_matrix'])

            R_g2b.append(ee_mat[:3, :3])
            t_g2b.append(ee_mat[:3, 3].reshape(3, 1))

            board_mat = rvec_tvec_to_matrix(s['board_rvec'], s['board_tvec'])
            R_t2c.append(board_mat[:3, :3])
            t_t2c.append(board_mat[:3, 3].reshape(3, 1))

        return R_g2b, t_g2b, R_t2c, t_t2c, skipped

    def _solve_one(self, samples, euler_conv):
        """在单一约定下求解并返回 (rot_err, trans_err, cam2ee_mat)。失败返回 None。"""
        R_g2b, t_g2b, R_t2c, t_t2c, skipped = self._build_pose_lists(
            samples, euler_conv)
        if len(R_g2b) < 3:
            return None
        R_cam2ee, t_cam2ee = solve_hand_eye(
            R_g2b, t_g2b, R_t2c, t_t2c, method=self.method)
        rot_err, trans_err = compute_reprojection_error(
            R_g2b, t_g2b, R_t2c, t_t2c, R_cam2ee, t_cam2ee)
        mat = np.eye(4)
        mat[:3, :3] = R_cam2ee
        mat[:3, 3] = t_cam2ee.flatten()
        return rot_err, trans_err, mat, skipped

    def _calibrate_callback(self, request, response):
        """执行手眼标定计算。"""
        samples, err = self._load_samples()
        if err:
            response.success = False
            response.message = err
            return response

        # --- Auto 模式：遍历所有约定 ---
        if self.euler_conv == 'auto':
            self.get_logger().info(
                f'AUTO mode: trying {len(AUTO_CONVENTIONS)} conventions...')
            results = []
            for conv in AUTO_CONVENTIONS:
                r = self._solve_one(samples, conv)
                if r is None:
                    self.get_logger().warn(
                        f'  [{conv}] skipped (no raw_pose)')
                    continue
                rot_err, trans_err, mat, skipped = r
                tmag = np.linalg.norm(mat[:3, 3]) * 1000  # mm
                results.append((conv, rot_err, trans_err, tmag, mat))
                self.get_logger().info(
                    f'  [{conv:>3}] rot={rot_err:7.3f}°  '
                    f'trans_err={trans_err:7.2f}mm  '
                    f'X.t_mag={tmag:6.1f}mm')

            if not results:
                response.success = False
                response.message = (
                    'AUTO failed: no samples had raw_pose field. '
                    'Need to re-capture with updated data_collector.')
                return response

            # 按组合指标选最佳: rot_err + trans_err/100
            results.sort(key=lambda r: r[1] + r[2] / 100.0)
            best_conv, best_rot, best_trans, best_tmag, best_mat = results[0]

            result_path = os.path.join(
                self.results_dir, f'{self.arm_name}_hand_eye.yaml')
            save_transform_yaml(result_path, best_mat, name='cam_to_ee')

            msg = (
                f'AUTO calibration complete for [{self.arm_name}].\n'
                f'  BEST convention: "{best_conv}"\n'
                f'  Rotation error:    {best_rot:.4f} deg\n'
                f'  Translation error: {best_trans:.4f} mm\n'
                f'  |X.translation|:   {best_tmag:.1f} mm (physical cam offset)\n'
                f'  Result saved: {result_path}\n'
                f'  Tip: set euler_convention="{best_conv}" in yaml for '
                f'future runs (and arm_driver).')
            self.get_logger().info(msg)
            response.success = True
            response.message = msg
            return response

        # --- 单一约定（或不指定） ---
        euler_conv = self.euler_conv or None
        if euler_conv:
            self.get_logger().info(
                f'Rebuilding EE pose from raw_pose with euler_convention="{euler_conv}"')

        r = self._solve_one(samples, euler_conv)
        if r is None:
            response.success = False
            response.message = (
                'Not enough usable samples. '
                'If using euler_convention, dataset must have raw_pose field.')
            return response

        rot_err, trans_err, mat, skipped = r
        if skipped > 0:
            self.get_logger().warn(
                f'{skipped} samples had no raw_pose, skipped.')

        tmag = np.linalg.norm(mat[:3, 3]) * 1000

        result_path = os.path.join(
            self.results_dir, f'{self.arm_name}_hand_eye.yaml')
        save_transform_yaml(result_path, mat, name='cam_to_ee')

        msg = (
            f'Calibration complete for [{self.arm_name}].\n'
            f'  euler_convention:  '
            f'{repr(euler_conv) if euler_conv else "(ee_pose_matrix)"}\n'
            f'  Rotation error:    {rot_err:.4f} deg\n'
            f'  Translation error: {trans_err:.4f} mm\n'
            f'  |X.translation|:   {tmag:.1f} mm (physical cam offset)\n'
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

