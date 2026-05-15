"""标定板 TF 发布节点

实时检测 ChArUco 标定板，将其位姿以 TF 的形式发布到 ROS2 树中。

发布的变换：
    camera_frame  →  board_frame   (动态，随检测结果更新)

配合 arm_driver 的 TF 链：
    base_link → ee_link → camera_link → charuco_board

用法：
    ros2 run calib_realman board_tf_publisher --ros-args \\
        -p camera_topic:=/camera_l/color/image_raw \\
        -p camera_info_topic:=/camera_l/color/camera_info \\
        -p camera_frame:=camera_l_link
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation

from .utils.charuco_detector import CharucoDetector
from .utils.transform_utils import rvec_tvec_to_matrix


class BoardTFPublisherNode(Node):
    def __init__(self):
        super().__init__('board_tf_publisher')

        # ── 参数 ──────────────────────────────────────────────────────────
        self.declare_parameter('camera_topic',      '/camera_l/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_l/color/camera_info')
        self.declare_parameter('camera_frame',      'camera_l_link')
        self.declare_parameter('board_frame',       'charuco_board')
        # 检测丢失时是否继续发布上一次有效位姿（保持 TF 树连通）
        self.declare_parameter('publish_last_when_lost', True)

        # ChArUco 板参数
        self.declare_parameter('squares_x',     5)
        self.declare_parameter('squares_y',     7)
        self.declare_parameter('square_length', 0.04)
        self.declare_parameter('marker_length', 0.03)
        self.declare_parameter('dictionary',    'DICT_5X5_100')

        camera_topic      = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.board_frame  = self.get_parameter('board_frame').value
        self.publish_last = self.get_parameter('publish_last_when_lost').value

        # ── ChArUco 检测器 ─────────────────────────────────────────────────
        self.charuco = CharucoDetector(
            squares_x=self.get_parameter('squares_x').value,
            squares_y=self.get_parameter('squares_y').value,
            square_length=self.get_parameter('square_length').value,
            marker_length=self.get_parameter('marker_length').value,
            dictionary_name=self.get_parameter('dictionary').value,
        )

        # ── 状态 ──────────────────────────────────────────────────────────
        self.bridge        = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs   = None
        self._last_rvec    = None   # 最后一次有效检测结果
        self._last_tvec    = None

        # ── TF 广播器 ──────────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── 订阅 ──────────────────────────────────────────────────────────
        self.create_subscription(Image,      camera_topic,      self._image_cb, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._info_cb,  10)

        self.get_logger().info(
            f'[BoardTFPublisher] camera_frame={self.camera_frame} '
            f'board_frame={self.board_frame}\n'
            f'  image : {camera_topic}\n'
            f'  info  : {camera_info_topic}\n'
            f'  publish_last_when_lost: {self.publish_last}')

    # ── 回调 ──────────────────────────────────────────────────────────────

    def _info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info('[BoardTFPublisher] Camera intrinsics received.')

    def _image_cb(self, msg):
        if self.camera_matrix is None:
            return

        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        stamp = msg.header.stamp

        rvec, tvec, _, _, ok = self.charuco.detect_and_estimate(
            image, self.camera_matrix, self.dist_coeffs)

        if ok:
            self._last_rvec = rvec
            self._last_tvec = tvec
            self._publish_tf(rvec, tvec, stamp)
        elif self.publish_last and self._last_rvec is not None:
            # 检测丢失，发布最后一次有效位姿（stamp 用当前时间保持 TF 树新鲜）
            self._publish_tf(self._last_rvec, self._last_tvec, stamp)

    def _publish_tf(self, rvec, tvec, stamp):
        """将 board 在 camera 坐标系下的位姿发布为 TF。"""
        # solvePnP 给出的是 T_board_in_cam（board 原点在 camera 坐标系下的位置和姿态）
        T = rvec_tvec_to_matrix(rvec, tvec)
        translation = T[:3, 3]
        quat = Rotation.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]

        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id  = self.board_frame
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)


# ── 入口 ──────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BoardTFPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
