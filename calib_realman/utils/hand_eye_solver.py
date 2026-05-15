"""手眼标定求解器封装"""

import cv2
import numpy as np
from .transform_utils import rvec_tvec_to_matrix, matrix_to_rvec_tvec


# OpenCV手眼标定方法映射
HAND_EYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
}


def solve_hand_eye(R_gripper2base_list, t_gripper2base_list,
                   R_target2cam_list, t_target2cam_list,
                   method="PARK"):
    """求解手眼标定 AX=XB (eye-in-hand)。

    对于eye-in-hand:
        A = gripper相对于base的变换 (从机械臂FK获得)
        B = 标定板相对于相机的变换 (从视觉检测获得)
        X = 相机相对于gripper(EE)的变换 (待求)

    Args:
        R_gripper2base_list: list of 3x3 rotation matrices (gripper->base)
        t_gripper2base_list: list of 3x1 translation vectors
        R_target2cam_list: list of 3x3 rotation matrices (target->camera)
        t_target2cam_list: list of 3x1 translation vectors
        method: 求解方法名称

    Returns:
        (R_cam2gripper, t_cam2gripper): 相机到末端执行器的变换
    """
    method_flag = HAND_EYE_METHODS.get(method)
    if method_flag is None:
        raise ValueError(f"Unknown method: {method}. Available: {list(HAND_EYE_METHODS.keys())}")

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base_list, t_gripper2base_list,
        R_target2cam_list, t_target2cam_list,
        method=method_flag
    )

    return R_cam2gripper, t_cam2gripper


def compute_reprojection_error(R_gripper2base_list, t_gripper2base_list,
                                R_target2cam_list, t_target2cam_list,
                                R_cam2gripper, t_cam2gripper):
    """计算手眼标定的一致性误差（AX = XB 残差）。

    Eye-in-hand 约束推导：
      T_g[k] @ X @ T_b[k] = T_board2world（常数），对第 i、j 对：

        T_g[j]⁻¹ @ T_g[i] @ X  =  X @ T_b[j] @ T_b[i]⁻¹
               A_ij                        B_ij

    因此：
        A_ij = inv(T_g[j]) @ T_g[i]  →  inv(Aj) @ Ai
        B_ij = T_b[j] @ inv(T_b[i])  →  Bj @ inv(Bi)   ← 注意 j 在前

    Returns:
        (rotation_error_deg, translation_error_mm): 平均旋转误差(度)和平移误差(mm)
    """
    n = len(R_gripper2base_list)
    rot_errors = []
    trans_errors = []

    X = np.eye(4)
    X[:3, :3] = R_cam2gripper
    X[:3, 3] = t_cam2gripper.flatten()

    for i in range(n):
        for j in range(i + 1, n):
            # A_ij = inv(T_g[j]) @ T_g[i]
            Ai = np.eye(4)
            Ai[:3, :3] = R_gripper2base_list[i]
            Ai[:3, 3] = t_gripper2base_list[i].flatten()

            Aj = np.eye(4)
            Aj[:3, :3] = R_gripper2base_list[j]
            Aj[:3, 3] = t_gripper2base_list[j].flatten()

            A_ij = np.linalg.inv(Aj) @ Ai

            # B_ij = T_b[j] @ inv(T_b[i])  （j 在前，i 在后，与 A 的 j/i 顺序对应）
            Bi = np.eye(4)
            Bi[:3, :3] = R_target2cam_list[i]
            Bi[:3, 3] = t_target2cam_list[i].flatten()

            Bj = np.eye(4)
            Bj[:3, :3] = R_target2cam_list[j]
            Bj[:3, 3] = t_target2cam_list[j].flatten()

            B_ij = Bj @ np.linalg.inv(Bi)   # 修复：原为 Bi @ inv(Bj)，i/j 对调导致算出 B⁻¹

            # AX vs XB
            AX = A_ij @ X
            XB = X @ B_ij

            # Rotation error
            R_err = AX[:3, :3].T @ XB[:3, :3]
            angle_err = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
            rot_errors.append(np.degrees(angle_err))

            # Translation error
            t_err = np.linalg.norm(AX[:3, 3] - XB[:3, 3])
            trans_errors.append(t_err * 1000)  # 转为mm

    avg_rot_err = np.mean(rot_errors) if rot_errors else 0.0
    avg_trans_err = np.mean(trans_errors) if trans_errors else 0.0

    return avg_rot_err, avg_trans_err
