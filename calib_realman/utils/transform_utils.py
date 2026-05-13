"""变换工具函数：矩阵构造、分解、转换"""

import numpy as np
from scipy.spatial.transform import Rotation


def pose_to_matrix(position, quaternion):
    """将位置+四元数转为4x4齐次变换矩阵。

    Args:
        position: [x, y, z]
        quaternion: [qx, qy, qz, qw] (ROS convention)
    """
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    mat[:3, 3] = position
    return mat


def matrix_to_pose(mat):
    """将4x4齐次变换矩阵分解为位置+四元数。

    Returns:
        (position, quaternion): position=[x,y,z], quaternion=[qx,qy,qz,qw]
    """
    position = mat[:3, 3].tolist()
    quaternion = Rotation.from_matrix(mat[:3, :3]).as_quat().tolist()
    return position, quaternion


def invert_transform(mat):
    """求逆变换矩阵。"""
    inv = np.eye(4)
    inv[:3, :3] = mat[:3, :3].T
    inv[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return inv


def compose_transforms(*transforms):
    """依次组合多个变换矩阵: T_total = T1 @ T2 @ ... @ Tn"""
    result = np.eye(4)
    for t in transforms:
        result = result @ t
    return result


def rvec_tvec_to_matrix(rvec, tvec):
    """OpenCV的rvec+tvec转4x4矩阵。"""
    import cv2
    R, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64).reshape(3, 1))
    mat = np.eye(4)
    mat[:3, :3] = R
    mat[:3, 3] = np.array(tvec, dtype=np.float64).flatten()
    return mat


def matrix_to_rvec_tvec(mat):
    """4x4矩阵转OpenCV的rvec+tvec。"""
    import cv2
    rvec, _ = cv2.Rodrigues(mat[:3, :3])
    tvec = mat[:3, 3].reshape(3, 1)
    return rvec, tvec


def save_transform_yaml(filepath, transform_mat, name="transform"):
    """将变换矩阵保存为YAML文件。"""
    import yaml
    position, quaternion = matrix_to_pose(transform_mat)
    data = {
        name: {
            'position': {'x': position[0], 'y': position[1], 'z': position[2]},
            'orientation': {
                'x': quaternion[0], 'y': quaternion[1],
                'z': quaternion[2], 'w': quaternion[3]
            },
            'matrix': transform_mat.tolist(),
        }
    }
    with open(filepath, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)


def load_transform_yaml(filepath, name="transform"):
    """从YAML文件加载变换矩阵。"""
    import yaml
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    return np.array(data[name]['matrix'])
