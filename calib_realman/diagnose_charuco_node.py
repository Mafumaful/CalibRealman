"""诊断工具：订阅相机图像，尝试所有ArUco字典，找出哪个能检测到标记。

用法:
    ros2 run calib_realman diagnose_charuco --ros-args \
        -p camera_topic:=/camera_l/color/image_raw

然后查看日志输出。会把带标注的图片保存到 /tmp/charuco_diag_*.png
"""

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


ALL_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class DiagnoseNode(Node):
    def __init__(self):
        super().__init__('diagnose_charuco')
        self.declare_parameter('camera_topic', '/camera_l/color/image_raw')
        self.declare_parameter('out_dir', '/tmp')
        self.declare_parameter('once', True)  # 检测一次后打印结果即退出

        camera_topic = self.get_parameter('camera_topic').value
        self.out_dir = self.get_parameter('out_dir').value
        self.once = self.get_parameter('once').value

        os.makedirs(self.out_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.done = False

        self.sub = self.create_subscription(
            Image, camera_topic, self._cb, 10)
        self.get_logger().info(f'Subscribing to {camera_topic}, waiting for image...')

    def _cb(self, msg):
        if self.done and self.once:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 保存原始图以供参考
        raw_path = os.path.join(self.out_dir, 'charuco_diag_raw.png')
        cv2.imwrite(raw_path, img)
        self.get_logger().info(f'Saved raw image: {raw_path}')
        self.get_logger().info(f'Image shape: {img.shape}')

        # 遍历所有字典
        self.get_logger().info('=== Testing all ArUco dictionaries ===')
        best_dict = None
        best_count = 0

        params = cv2.aruco.DetectorParameters()
        for name, dict_id in ALL_DICTS.items():
            d = cv2.aruco.getPredefinedDictionary(dict_id)
            detector = cv2.aruco.ArucoDetector(d, params)
            corners, ids, _ = detector.detectMarkers(gray)

            n = 0 if ids is None else len(ids)
            if n > 0:
                self.get_logger().info(
                    f'  [{name}] => {n} markers detected, IDs: {ids.flatten().tolist()}')
                # 保存标注图
                vis = img.copy()
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                vis_path = os.path.join(
                    self.out_dir, f'charuco_diag_{name}.png')
                cv2.imwrite(vis_path, vis)
                if n > best_count:
                    best_count = n
                    best_dict = name

        if best_dict is None:
            self.get_logger().error(
                'NO ArUco markers detected with ANY dictionary!\n'
                'Possible causes:\n'
                '  1. Camera image too blurry/dark/over-exposed\n'
                '  2. Board too far or too close\n'
                '  3. Board not fully in view\n'
                f'  4. Check raw image: {raw_path}')
        else:
            self.get_logger().info(
                f'=== BEST MATCH: {best_dict} with {best_count} markers ===\n'
                f'Update config/charuco_board.yaml:\n'
                f'    dictionary: "{best_dict}"')

        self.done = True
        if self.once:
            self.get_logger().info('Diagnosis complete. Shutting down.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DiagnoseNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
