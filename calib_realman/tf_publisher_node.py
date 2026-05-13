"""TF发布节点：读取所有标定结果，发布完整的静态TF树。"""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from .utils.transform_utils import load_transform_yaml, matrix_to_pose, pose_to_matrix


class TFPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        self.declare_parameter('results_dir', 'results')
        # 世界坐标系定义 [x,y,z,qx,qy,qz,qw]
        self.declare_parameter(
            'arm1_base_to_world', [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter(
            'arm2_base_to_world', [-0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter(
            'board_to_world', [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0])

        self.results_dir = self.get_parameter('results_dir').value
        arm1_param = self.get_parameter('arm1_base_to_world').value
        arm2_param = self.get_parameter('arm2_base_to_world').value
        board_param = self.get_parameter('board_to_world').value

        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = []

        # world -> arm1_base_link
        transforms.append(self._make_tf(
            'world', 'arm1_base_link', arm1_param[:3], arm1_param[3:]))

        # world -> arm2_base_link
        transforms.append(self._make_tf(
            'world', 'arm2_base_link', arm2_param[:3], arm2_param[3:]))

        # world -> charuco_board
        transforms.append(self._make_tf(
            'world', 'charuco_board', board_param[:3], board_param[3:]))

        # arm1_ee_link -> arm1_cam_link (from hand-eye calibration)
        arm1_he_path = os.path.join(self.results_dir, 'arm1_hand_eye.yaml')
        if os.path.exists(arm1_he_path):
            mat = load_transform_yaml(arm1_he_path, 'cam_to_ee')
            pos, quat = matrix_to_pose(mat)
            transforms.append(self._make_tf(
                'arm1_ee_link', 'arm1_cam_link', pos, quat))
            self.get_logger().info('Loaded arm1 hand-eye calibration.')
        else:
            self.get_logger().warn(f'arm1 hand-eye result not found: {arm1_he_path}')

        # arm2_ee_link -> arm2_cam_link (from hand-eye calibration)
        arm2_he_path = os.path.join(self.results_dir, 'arm2_hand_eye.yaml')
        if os.path.exists(arm2_he_path):
            mat = load_transform_yaml(arm2_he_path, 'cam_to_ee')
            pos, quat = matrix_to_pose(mat)
            transforms.append(self._make_tf(
                'arm2_ee_link', 'arm2_cam_link', pos, quat))
            self.get_logger().info('Loaded arm2 hand-eye calibration.')
        else:
            self.get_logger().warn(f'arm2 hand-eye result not found: {arm2_he_path}')

        # world -> global_cam_link (from global camera calibration)
        global_path = os.path.join(self.results_dir, 'global_cam_to_world.yaml')
        if os.path.exists(global_path):
            mat = load_transform_yaml(global_path, 'global_cam_to_world')
            pos, quat = matrix_to_pose(mat)
            transforms.append(self._make_tf(
                'world', 'global_cam_link', pos, quat))
            self.get_logger().info('Loaded global camera calibration.')
        else:
            self.get_logger().warn(f'Global cam result not found: {global_path}')

        # 发布所有静态TF
        if transforms:
            self.broadcaster.sendTransform(transforms)
            self.get_logger().info(
                f'Published {len(transforms)} static transforms.')

    def _make_tf(self, parent_frame, child_frame, position, quaternion):
        """构建TransformStamped消息。"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        return t


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
