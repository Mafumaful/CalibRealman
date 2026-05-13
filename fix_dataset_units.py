"""修复现有 dataset.json 的单位问题。

睿尔曼机械臂返回的位置单位是毫米，但手眼标定需要米（与OpenCV solvePnP tvec一致）。
此脚本将 dataset.json 中所有样本的位置字段除以 1000（mm -> m）。

会修复:
  - sample['ee_pose_matrix'] 的平移部分 [:3, 3]
  - sample['raw_pose'] 的前三个元素 [0:3]

原文件备份为 dataset.json.bak

用法:
    python3 fix_dataset_units.py calibration_data/arm1/dataset.json
    python3 fix_dataset_units.py calibration_data/arm2/dataset.json
"""

import json
import shutil
import sys


GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
RESET = '\033[0m'


def fix_dataset(path, scale=0.001):
    print(f'{YELLOW}[1/3] 备份原文件...{RESET}')
    shutil.copy(path, path + '.bak')
    print(f'  Backup: {path}.bak')

    print(f'{YELLOW}[2/3] 加载并转换单位 (scale={scale})...{RESET}')
    with open(path, 'r') as f:
        data = json.load(f)

    n_ee = 0
    n_raw = 0
    first_before = None
    first_after = None

    for s in data.get('samples', []):
        # 修复 ee_pose_matrix 平移部分
        mat = s.get('ee_pose_matrix')
        if mat is not None:
            if first_before is None:
                first_before = [mat[0][3], mat[1][3], mat[2][3]]
            mat[0][3] *= scale
            mat[1][3] *= scale
            mat[2][3] *= scale
            if first_after is None:
                first_after = [mat[0][3], mat[1][3], mat[2][3]]
            n_ee += 1

        # 修复 raw_pose 前三个
        raw = s.get('raw_pose')
        if raw is not None and len(raw) >= 3:
            raw[0] *= scale
            raw[1] *= scale
            raw[2] *= scale
            n_raw += 1

    print(f'  Scaled {n_ee} ee_pose_matrix, {n_raw} raw_pose')
    if first_before:
        print(f'  Sample 0 before: {[f"{v:.4f}" for v in first_before]}')
        print(f'  Sample 0 after : {[f"{v:.4f}" for v in first_after]}')

    print(f'{YELLOW}[3/3] 保存...{RESET}')
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f'{GREEN}[OK]{RESET} Saved: {path}')
    print(f'{GREEN}重新运行标定即可:{RESET}')
    print(f'  ros2 service call /calibration/run_calibration std_srvs/srv/Trigger')


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 fix_dataset_units.py <dataset.json> [scale]')
        print('  scale default 0.001 (mm -> m)')
        sys.exit(1)

    path = sys.argv[1]
    scale = float(sys.argv[2]) if len(sys.argv) > 2 else 0.001

    print(f'Fix dataset units: {path} (scale={scale})')
    print()

    try:
        fix_dataset(path, scale)
    except Exception as e:
        print(f'{RED}[FAIL] {e}{RESET}')
        sys.exit(1)


if __name__ == '__main__':
    main()
