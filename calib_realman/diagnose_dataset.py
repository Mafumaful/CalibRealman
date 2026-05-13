"""数据集诊断：分析采集的样本多样性，判断手眼标定失败的原因。

用法:
    python3 -m calib_realman.diagnose_dataset calibration_data/arm1/dataset.json
或:
    ros2 run calib_realman diagnose_dataset calibration_data/arm1/dataset.json
"""

import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation


def analyze(dataset_path):
    with open(dataset_path, 'r') as f:
        data = json.load(f)

    samples = data['samples']
    n = len(samples)
    print(f'\n{"=" * 60}')
    print(f'  Dataset Diagnosis: {dataset_path}')
    print(f'  arm: {data["arm_name"]}, samples: {n}')
    print(f'{"=" * 60}\n')

    if n < 3:
        print(f'[FATAL] Need at least 3 samples, got {n}.')
        return

    # === 1. 末端位姿分析 ===
    ee_positions = []
    ee_eulers = []
    for s in samples:
        mat = np.array(s['ee_pose_matrix'])
        ee_positions.append(mat[:3, 3])
        # 转为 XYZ 欧拉角（度）方便人眼读
        ee_eulers.append(Rotation.from_matrix(mat[:3, :3]).as_euler('xyz', degrees=True))
    ee_positions = np.array(ee_positions)
    ee_eulers = np.array(ee_eulers)

    print('--- End-Effector Position (base frame, m) ---')
    for axis, name in enumerate(['x', 'y', 'z']):
        lo, hi = ee_positions[:, axis].min(), ee_positions[:, axis].max()
        print(f'  {name}: [{lo:+.3f}, {hi:+.3f}]  range={hi-lo:.3f} m')

    print('\n--- End-Effector Orientation (XYZ Euler, degrees) ---')
    for axis, name in enumerate(['rx', 'ry', 'rz']):
        lo, hi = ee_eulers[:, axis].min(), ee_eulers[:, axis].max()
        rng = hi - lo
        flag = '  [OK]' if rng > 30 else '  [!! TOO SMALL, need >30°]'
        print(f'  {name}: [{lo:+.1f}, {hi:+.1f}]  range={rng:.1f}°{flag}')

    # 两两之间的最大旋转差（最重要的诊断指标）
    max_pair_angle = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            Ri = np.array(samples[i]['ee_pose_matrix'])[:3, :3]
            Rj = np.array(samples[j]['ee_pose_matrix'])[:3, :3]
            R_rel = Ri.T @ Rj
            ang = np.degrees(np.arccos(np.clip((np.trace(R_rel) - 1) / 2, -1, 1)))
            max_pair_angle = max(max_pair_angle, ang)
    print(f'\n  Max pairwise rotation diff: {max_pair_angle:.1f}°')
    if max_pair_angle < 30:
        print('  [!! TOO SMALL] Hand-eye calibration will FAIL.')
        print('     → Need poses with >30° relative rotation between them.')
    elif max_pair_angle < 60:
        print('  [WARNING] Low rotation diversity, may give unstable results.')
    else:
        print('  [OK] Enough rotation diversity.')

    # === 2. 标定板位姿分析 ===
    print('\n--- Board Position (camera frame, m) ---')
    tvecs = np.array([s['board_tvec'] for s in samples])
    distances = np.linalg.norm(tvecs, axis=1)
    print(f'  distance: [{distances.min():.3f}, {distances.max():.3f}]  '
          f'mean={distances.mean():.3f} m')
    if distances.mean() < 0.15:
        print('  [WARNING] Board very close. D435 recommended 0.3-0.8m.')
    if distances.mean() > 1.0:
        print('  [WARNING] Board very far. Corner accuracy may be poor.')

    # === 3. 角点数分布 ===
    corners = [s['corners_detected'] for s in samples]
    print(f'\n--- ChArUco Corners Detected ---')
    print(f'  min/mean/max: {min(corners)}/{np.mean(corners):.1f}/{max(corners)}')
    low_count = sum(1 for c in corners if c < 15)
    if low_count > 0:
        print(f'  [WARNING] {low_count}/{n} samples with <15 corners. '
              f'Consider re-capturing those.')

    # === 4. 样本数 ===
    print(f'\n--- Sample Count ---')
    if n < 8:
        print(f'  [!! TOO FEW] {n} samples. Recommend >=15.')
    elif n < 15:
        print(f'  [WARNING] {n} samples. Recommend >=15 for stable result.')
    else:
        print(f'  [OK] {n} samples.')

    print(f'\n{"=" * 60}\n')


def main(args=None):
    if len(sys.argv) < 2:
        print('Usage: diagnose_dataset <path_to_dataset.json>')
        sys.exit(1)
    analyze(sys.argv[1])


if __name__ == '__main__':
    main()
