"""数据采集节点：采集手眼标定所需的图像+末端位姿数据对。"""

import os
import json
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
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

        self.declare_parameter('calibration.output_dir', 'calibration_data')
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
        output_base = self.get_parameter('calibration.output_dir').value

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
        self.latest_raw_pose = None  # 睿尔曼原始 [x,y,z,rx,ry,rz]
        self.samples = []

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅
        self.image_sub = self.create_subscription(
            Image, camera_topic, self._image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._info_callback, 10)
        # 订阅原始位姿（供后续用不同欧拉约定重算）
        self.raw_pose_sub = self.create_subscription(
            Float64MultiArray, f'/{self.arm_name}/raw_pose',
            self._raw_pose_callback, 10)

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

        # 启动时打印所有加载的参数
        self._print_loaded_params(
            camera_topic, camera_info_topic, output_base, preview_rate)

        self.get_logger().info(
            f'Data collector ready for [{self.arm_name}]. '
            f'Call ~/capture to collect samples.')

    def _print_loaded_params(self, camera_topic, camera_info_topic,
                             output_base, preview_rate):
        """启动时打印所有加载的配置参数，便于确认。"""
        lines = [
            '=' * 60,
            f'  Data Collector Loaded Parameters [{self.arm_name}]',
            '=' * 60,
            f'  arm_name          : {self.arm_name}',
            f'  camera_topic      : {camera_topic}',
            f'  camera_info_topic : {camera_info_topic}',
            f'  base_frame        : {self.base_frame}',
            f'  ee_frame          : {self.ee_frame}',
            f'  output_dir        : {output_base}/{self.arm_name}',
            f'  preview_rate      : {preview_rate} Hz',
            '  --- ChArUco Board ---',
            f'  squares_x         : {self.get_parameter("squares_x").value}',
            f'  squares_y         : {self.get_parameter("squares_y").value}',
            f'  square_length (m) : {self.get_parameter("square_length").value}',
            f'  marker_length (m) : {self.get_parameter("marker_length").value}',
            f'  dictionary        : {self.get_parameter("dictionary").value}',
            '=' * 60,
        ]
        for line in lines:
            self.get_logger().info(line)

    def _image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received.')

    def _raw_pose_callback(self, msg):
        if len(msg.data) >= 6:
            self.latest_raw_pose = list(msg.data[:6])

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
        """定时打印当前帧的ChArUco检测状态（区分ArUco与ChArUco两层）。"""
        if self.latest_image is None:
            return
        n_aruco, n_charuco = self.charuco.detect_diagnostic(self.latest_image)
        if n_aruco == 0:
            self.get_logger().warn(
                '[PREVIEW] No ArUco markers detected. '
                'Likely WRONG DICTIONARY. Run: '
                'ros2 run calib_realman diagnose_charuco --ros-args '
                f'-p camera_topic:={self.image_sub.topic_name}')
        elif n_charuco < 6:
            self.get_logger().warn(
                f'[PREVIEW] ArUco markers={n_aruco} but ChArUco corners={n_charuco} (<6). '
                'Likely WRONG board geometry (squares_x/squares_y) or partial occlusion.')
        else:
            self.get_logger().info(
                f'[PREVIEW] ArUco={n_aruco}, ChArUco corners={n_charuco}. '
                'Ready to capture.')

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
            # 原始睿尔曼位姿 [x,y,z,rx,ry,rz] (m + rad)，未做欧拉约定转换
            # 标定时可指定 euler_convention 参数从这里重算 ee_pose_matrix
            'raw_pose': self.latest_raw_pose,
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
