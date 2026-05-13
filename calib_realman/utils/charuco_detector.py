"""ChArUco标定板检测封装"""

import cv2
import numpy as np


class CharucoDetector:
    """ChArUco标定板检测器。"""

    # OpenCV ArUco字典映射
    DICT_MAP = {
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
    }

    def __init__(self, squares_x, squares_y, square_length, marker_length, dictionary_name):
        """
        Args:
            squares_x: 棋盘格列数
            squares_y: 棋盘格行数
            square_length: 方格边长 (米)
            marker_length: ArUco标记边长 (米)
            dictionary_name: 字典名称，如 "DICT_4X4_50"
        """
        self.squares_x = squares_x
        self.squares_y = squares_y
        self.square_length = square_length
        self.marker_length = marker_length

        dict_id = self.DICT_MAP.get(dictionary_name)
        if dict_id is None:
            raise ValueError(f"Unknown dictionary: {dictionary_name}")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_length, marker_length, self.aruco_dict
        )
        self.detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        self.charuco_detector = cv2.aruco.CharucoDetector(self.board)

    def detect(self, image):
        """检测ChArUco角点。

        Args:
            image: BGR图像 (numpy array)

        Returns:
            (charuco_corners, charuco_ids) 或 (None, None) 如果检测失败
        """
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.charuco_detector.detectBoard(image)
        )

        if charuco_corners is None or len(charuco_corners) < 6:
            return None, None

        return charuco_corners, charuco_ids

    def estimate_pose(self, charuco_corners, charuco_ids, camera_matrix, dist_coeffs):
        """估计标定板相对于相机的位姿。

        Returns:
            (rvec, tvec, success): 旋转向量、平移向量、是否成功
        """
        if charuco_corners is None or len(charuco_corners) < 6:
            return None, None, False

        obj_points, img_points = self.board.matchImagePoints(charuco_corners, charuco_ids)

        if obj_points is None or len(obj_points) < 6:
            return None, None, False

        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, camera_matrix, dist_coeffs
        )

        if not success:
            return None, None, False

        return rvec, tvec, True

    def detect_and_estimate(self, image, camera_matrix, dist_coeffs):
        """一步完成检测+位姿估计。

        Returns:
            (rvec, tvec, corners, ids, success)
        """
        corners, ids = self.detect(image)
        if corners is None:
            return None, None, None, None, False

        rvec, tvec, success = self.estimate_pose(corners, ids, camera_matrix, dist_coeffs)
        return rvec, tvec, corners, ids, success

    def draw_detected(self, image, corners, ids):
        """在图像上绘制检测到的角点。"""
        vis = image.copy()
        if corners is not None and ids is not None:
            cv2.aruco.drawDetectedCornersCharuco(vis, corners, ids)
        return vis
