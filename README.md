# CalibRealman

睿尔曼 RM65 双臂手眼标定工具 (ROS2 Humble)

## 系统配置

- 2x Realman RM65 机械臂，各配 1x Intel RealSense D435 (eye-in-hand)
- 1x Intel RealSense D435 固定全局相机
- ChArUco 标定板（固定位置）

## 坐标系关系

```
world (两臂中点)
├── arm1_base_link
│   └── arm1_ee_link (FK)
│       └── arm1_cam_link (手眼标定)
├── arm2_base_link
│   └── arm2_ee_link (FK)
│       └── arm2_cam_link (手眼标定)
├── global_cam_link (全局相机标定)
└── charuco_board (已知/配置)
```

## 使用流程

### 1. 配置参数

编辑 `config/charuco_board.yaml` 填写标定板参数，编辑 `config/calibration_params.yaml` 填写相机序列号和臂间距。

### 2. 启动相机

```bash
ros2 launch calib_realman cameras.launch.py \
  arm1_serial:=<SN1> arm2_serial:=<SN2> global_serial:=<SN3>
```

### 3. 数据采集

```bash
# 采集 arm1 数据
ros2 launch calib_realman collect_data.launch.py arm_name:=arm1

# 采集 arm2 数据
ros2 launch calib_realman collect_data.launch.py arm_name:=arm2
```

操作：手动示教机械臂到不同位姿，按空格键采集，按 s 保存。

### 4. 执行标定

```bash
ros2 launch calib_realman calibrate.launch.py
```

然后调用服务触发计算：
```bash
ros2 service call /calibration_arm1/run_calibration std_srvs/srv/Trigger
ros2 service call /calibration_arm2/run_calibration std_srvs/srv/Trigger
ros2 service call /global_cam_calibration/calibrate_global std_srvs/srv/Trigger
```

### 5. 发布 TF

```bash
ros2 launch calib_realman publish_tf.launch.py
```

## 标定板参考

见 `assets/charuco_board_A4.pdf`

## 依赖

- ROS2 Humble
- realsense2_camera
- opencv-contrib-python
- numpy, scipy, transforms3d
