# CalibRealman

睿尔曼 RM65 双臂手眼标定工具 (ROS2 Humble)

## 系统配置

- 2x Realman RM65 机械臂，各配 1x Intel RealSense D435 (eye-in-hand)
- 1x Intel RealSense D435 固定全局相机
- ChArUco 标定板（固定位置）

## 话题命名约定

- `camera_l/*` — 左臂末端相机 (arm1)
- `camera_r/*` — 右臂末端相机 (arm2)
- `camera_m/*` — 中间全局相机

> 若实际话题名带序列号后缀（如 `camera_r_305`），请在 `config/calibration_params.yaml` 中调整。

## 坐标系关系

```
world (两臂中点)
├── arm1_base_link → arm1_ee_link (FK) → camera_l_link (手眼标定)
├── arm2_base_link → arm2_ee_link (FK) → camera_r_link (手眼标定)
├── camera_m_link (全局相机标定)
└── charuco_board
```

## 使用流程

### 1. 配置参数

- 编辑 `config/charuco_board.yaml` 填写标定板实际参数
- 编辑 `config/calibration_params.yaml` 根据实际话题名调整，填写臂间距

### 2. 启动相机与机械臂（用户自行启动）

用户自行启动三个 RealSense 相机和两个 Realman 驱动，确保：
- `/camera_l/*`、`/camera_r/*`、`/camera_m/*` 话题可用
- TF 树可查询 `arm1_base_link -> arm1_ee_link` 和 `arm2_base_link -> arm2_ee_link`

### 3. 数据采集

**终端 A** 启动数据采集节点：
```bash
# 采集 arm1（左臂）
ros2 launch calib_realman collect_data.launch.py arm_name:=arm1
```

**终端 B** 启动键盘触发器（必须在单独终端，需要真实 TTY）：
```bash
ros2 run calib_realman keyboard_trigger
```

操作：手动示教机械臂到不同位姿（建议 15-20 个，覆盖不同角度和位置），按 **空格键** 采集，采集完按 **s** 保存数据集，**q** 退出。

对 `arm_name:=arm2` 重复上述流程。

### 4. 执行标定

```bash
ros2 launch calib_realman calibrate.launch.py
```

调用服务触发计算：
```bash
ros2 service call /calibration_arm1/run_calibration std_srvs/srv/Trigger
ros2 service call /calibration_arm2/run_calibration std_srvs/srv/Trigger
ros2 service call /global_cam_calibration/calibrate_global std_srvs/srv/Trigger
```

### 5. 发布 TF

```bash
ros2 launch calib_realman publish_tf.launch.py
```

之后可通过 `ros2 run tf2_tools view_frames` 查看完整 TF 树。

## 标定板参考

见 `assets/charuco_board_A4.pdf`

## 依赖

- ROS2 Humble
- opencv-contrib-python
- numpy, scipy, transforms3d, pyyaml
