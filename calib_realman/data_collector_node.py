"""数据采集节点：采集手眼标定所需的图像+末端位姿数据对。"""

import os
import json
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped

from .utils.charuco_detector import CharucoDetector
from .utils.transform_utils import pose_to_matrix, matrix_to_pose


class DataCollectorNode(Node):
    def __init__(self):
        super().__init__('data_collector')

        # 先声明 arm_name，根据它读取对应的相机/坐标系参数
        self.declare_parameter('arm_name', 'arm1')
        self.arm_name = self.get_parameter('arm_name').value

        # 嵌套参数: <arm_name>.camera_topic 等
        prefix = self.arm_name
        self.declare_parameter(f'{prefix}.camera_topic',
                               f'/camera_l/color/image_raw')
        self.declare_parameter(f'{prefix}.camera_info_topic',
                               f'/camera_l/color/camera_info')
        self.declare_parameter(f'{prefix}.base_frame', f'{prefix}_base_link')
        self.declare_parameter(f'{prefix}.ee_frame', f'{prefix}_ee_link')

        self.declare_parameter('output_dir', 'calibration_data')
        # ChArUco参数
        self.declare_parameter('squares_x', 5)
        self.declare_parameter('squares_y', 7)
        self.declare_parameter('square_length', 0.04)
        self.declare_parameter('marker_length', 0.03)
        self.declare_parameter('dictionary', 'DICT_4X4_50')

        # 获取参数
        camera_topic = self.get_parameter(f'{prefix}.camera_topic').value
        camera_info_topic = self.get_parameter(f'{prefix}.camera_info_topic').value
        self.base_frame = self.get_parameter(f'{prefix}.base_frame').value
        self.ee_frame = self.get_parameter(f'{prefix}.ee_frame').value
        output_base = self.get_parameter('output_dir').value

        # 输出目录
        self.output_dir = os.path.join(output_base, self.arm_name)
        self.images_dir = os.path.join(self.output_dir, 'images')
        self.vis_dir = os.path.join(self.output_dir, 'vis')
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.vis_dir, exist_ok=True)

        # ChArUco检测器
        self.charuco = CharucoDetector(
            squares_x=self.get_parameter('squares_x').value,
            squares_y=self.get_parameter('squares_y').value,
            square_length=self.get_parameter('square_length').value,
            marker_length=self.get_parameter('marker_length').value,
            dictionary_name=self.get_parameter('dictionary').value,
        )

        # 状态
        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.samples = []

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅
        self.image_sub = self.create_subscription(
            Image, camera_topic, self._image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._info_callback, 10)

        # 服务
        self.capture_srv = self.create_service(
            Trigger, '~/capture', self._capture_callback)
        self.save_srv = self.create_service(
            Trigger, '~/save_dataset', self._save_callback)

        # 实时预览定时器（每秒打印一次当前帧角点检测情况）
        self.declare_parameter('preview_rate', 1.0)
        preview_rate = self.get_parameter('preview_rate').value
        if preview_rate > 0:
            self.preview_timer = self.create_timer(
                1.0 / preview_rate, self._preview_callback)

        self.get_logger().info(
            f'Data collector ready for [{self.arm_name}]. '
            f'Call ~/capture to collect samples.')

    def _image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received.')

    def _get_ee_pose(self):
        """通过TF获取末端位姿 (base->ee)。"""
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time())
            pos = [t.transform.translation.x,
                   t.transform.translation.y,
                   t.transform.translation.z]
            quat = [t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w]
            return pose_to_matrix(pos, quat)
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

    def _preview_callback(self):
        """定时打印当前帧的ChArUco检测状态。"""
        if self.latest_image is None or self.camera_matrix is None:
            return
        corners, ids = self.charuco.detect(self.latest_image)
        if corners is None:
            self.get_logger().info('[PREVIEW] No ChArUco corners detected.')
        else:
            self.get_logger().info(
                f'[PREVIEW] Detected {len(corners)} corners '
                f'(need >=6 for capture).')

    def _capture_callback(self, request, response):
        """触发一次数据采集。"""
        if self.latest_image is None:
            response.success = False
            response.message = 'No image received yet.'
            return response

        if self.camera_matrix is None:
            response.success = False
            response.message = 'Camera intrinsics not received yet.'
            return response

        # 检测ChArUco
        rvec, tvec, corners, ids, success = self.charuco.detect_and_estimate(
            self.latest_image, self.camera_matrix, self.dist_coeffs)

        if not success:
            response.success = False
            response.message = 'ChArUco board not detected (need >= 6 corners).'
            return response

        # 获取末端位姿
        ee_mat = self._get_ee_pose()
        if ee_mat is None:
            response.success = False
            response.message = 'Cannot get end-effector pose from TF.'
            return response

        # 保存样本
        sample_id = len(self.samples)
        img_path = os.path.join(self.images_dir, f'sample_{sample_id:03d}.png')
        cv2.imwrite(img_path, self.latest_image)

        # 保存带角点标注的可视化图
        vis = self.charuco.draw_detected(self.latest_image, corners, ids)
        cv2.drawFrameAxes(
            vis, self.camera_matrix, self.dist_coeffs,
            rvec, tvec, 0.05)
        vis_path = os.path.join(self.vis_dir, f'sample_{sample_id:03d}_vis.png')
        cv2.imwrite(vis_path, vis)

        # 计算标定板距离和姿态
        distance = float(np.linalg.norm(tvec))
        rvec_deg = np.degrees(np.linalg.norm(rvec))

        sample = {
            'sample_id': sample_id,
            'timestamp': datetime.now().isoformat(),
            'ee_pose_matrix': ee_mat.tolist(),
            'board_rvec': rvec.flatten().tolist(),
            'board_tvec': tvec.flatten().tolist(),
            'corners_detected': len(corners),
            'image_path': img_path,
            'vis_path': vis_path,
        }
        self.samples.append(sample)

        response.success = True
        response.message = (
            f'Sample {sample_id} captured. '
            f'Corners: {len(corners)}, '
            f'Board dist: {distance:.3f}m, '
            f'Rot: {rvec_deg:.1f}deg. '
            f'Total: {len(self.samples)}')
        self.get_logger().info(response.message)
        self.get_logger().info(f'  Image:  {img_path}')
        self.get_logger().info(f'  Vis:    {vis_path}')
        return response

    def _save_callback(self, request, response):
        """保存所有采集数据到磁盘。"""
        if not self.samples:
            response.success = False
            response.message = 'No samples collected.'
            return response

        dataset_path = os.path.join(self.output_dir, 'dataset.json')
        data = {
            'arm_name': self.arm_name,
            'num_samples': len(self.samples),
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'samples': self.samples,
        }
        with open(dataset_path, 'w') as f:
            json.dump(data, f, indent=2)

        response.success = True
        response.message = f'Dataset saved: {dataset_path} ({len(self.samples)} samples)'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
