# CalibRealman

睿尔曼双臂手眼标定工具 (ROS2 Humble，Python)

## 系统配置

- 2x Realman 机械臂（默认 RM65，SDK 自动识别型号与 DOF），各配 1x RealSense D435 (eye-in-hand)
- 1x RealSense D435 固定全局相机（两臂中间）
- ChArUco 标定板（固定放置）
- 通过 IP 连接机械臂（默认 arm1=192.168.30.123, arm2=192.168.30.124）

## 话题命名约定

- `/camera_l/*` — 左臂末端相机 (arm1)
- `/camera_r/*` — 右臂末端相机 (arm2)
- `/camera_m/*` — 中间全局相机
- `/arm1/joint_states`, `/arm2/joint_states` — 机械臂关节状态（由本包发布）

> 若实际话题名不同（如 `camera_r_305`），请在 `config/calibration_params.yaml` 里改 `camera_topic` / `camera_info_topic`。

## 坐标系关系

```
world (两臂中点)
├── arm1_base_link → arm1_ee_link (驱动动态发布) → camera_l_link (手眼标定)
├── arm2_base_link → arm2_ee_link (驱动动态发布) → camera_r_link (手眼标定)
├── camera_m_link (全局相机标定)
└── charuco_board
```

## 节点概览

| 节点 | 可执行名 | 作用 |
|------|----------|------|
| 机械臂驱动 | `arm_driver` | 通过 IP 连睿尔曼机械臂，发布 `arm*_base_link → arm*_ee_link` TF 和 `/<arm>/joint_states` |
| 数据采集 | `data_collector` | 订阅相机图像 + 查询 TF，触发式采集手眼标定数据对 |
| 标定计算 | `calibration` | 加载采集数据，调用 `cv2.calibrateHandEye` 计算 cam→ee 变换 |
| 全局相机标定 | `global_cam` | 用全局相机观测标定板，求解 `global_cam → world` |
| TF 发布 | `tf_publisher` | 读取所有标定结果，发布完整静态 TF 树 |
| 诊断工具 | `diagnose_charuco` | 遍历所有 ArUco 字典，找出标定板实际用的那个 |
| 键盘触发 | `keyboard_trigger` | 空格采集 / s 保存 / q 退出（需真实 TTY） |

## 依赖安装

```bash
# ROS2 Humble（略）

# 睿尔曼 Python SDK
pip install Robotic_Arm

# OpenCV with ArUco
pip install opencv-contrib-python

# 其他
pip install numpy scipy transforms3d pyyaml
```

## 使用流程

### 1. 配置参数

编辑两个配置文件：

**`config/calibration_params.yaml`** — 机械臂 IP、话题名、世界坐标系定义
```yaml
arm1:
  ip: "192.168.30.123"
  port: 8080
  camera_topic: "/camera_l/color/image_raw"
  ...
world_frame:
  arm1_base_to_world: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  arm2_base_to_world: [-0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  board_to_world: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0]
```

**`config/charuco_board.yaml`** — 标定板参数（字典、行列、尺寸）
```yaml
/**:
  ros__parameters:
    squares_x: 5
    squares_y: 7
    square_length: 0.04
    marker_length: 0.03
    dictionary: "DICT_4X4_50"
```

> 不确定字典类型？见「诊断工具」一节。

### 2. 构建

```bash
cd <your_ros2_ws>
colcon build --packages-select calib_realman
source install/setup.bash
```

### 3. 启动相机（用户自行启动）

自行启动三个 RealSense，确保 `/camera_l/*`、`/camera_r/*`、`/camera_m/*` 有图像。

### 4. 启动机械臂驱动（发布 base→ee 动态 TF）

两个终端分别：
```bash
ros2 launch calib_realman arm_driver.launch.py arm_name:=arm1
ros2 launch calib_realman arm_driver.launch.py arm_name:=arm2
```
启动日志会用**绿色**打印 `[CONNECT OK]`，**红色**打印 `[CONNECT FAIL]`。

验证：
```bash
ros2 run tf2_ros tf2_echo arm1_base_link arm1_ee_link
ros2 topic hz /tf
```

### 5. 数据采集

**终端 A** 启动数据采集节点：
```bash
ros2 launch calib_realman collect_data.launch.py arm_name:=arm1
```
节点会打印所有加载的参数（ChArUco 字典、话题名、坐标系名等），并每秒打印一次 `[PREVIEW]` 显示当前视野里检测到的 ArUco / ChArUco 角点数。

**终端 B** 启动键盘触发器（必须在真实终端）：
```bash
ros2 run calib_realman keyboard_trigger
```

操作：
- 手动示教机械臂到不同位姿（建议 15-20 个，覆盖不同角度、距离、方向）
- 每个位姿按 **空格** 采集一次；
- 采集完按 **s** 保存数据集到 `calibration_data/<arm>/dataset.json`；
- 按 **q** 退出。

每次采集的图像保存到 `calibration_data/<arm>/images/`，带角点标注的可视化图保存到 `calibration_data/<arm>/vis/`。

对 `arm_name:=arm2` 重复上述流程。

### 6. 执行标定

```bash
ros2 launch calib_realman calibrate.launch.py
```

触发计算：
```bash
ros2 service call /calibration_arm1/run_calibration std_srvs/srv/Trigger
ros2 service call /calibration_arm2/run_calibration std_srvs/srv/Trigger
ros2 service call /global_cam_calibration/calibrate_global std_srvs/srv/Trigger
```
结果保存到 `results/`：
- `arm1_hand_eye.yaml`、`arm2_hand_eye.yaml`（cam→ee 变换）
- `global_cam_to_world.yaml`、`board_to_world.yaml`

### 7. 发布标定 TF

```bash
ros2 launch calib_realman publish_tf.launch.py
```
之后：
```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf，可视化完整 TF 树
```

## 诊断工具

### 检查 ArUco 字典

如果 `data_collector` 一直提示 `No ArUco markers detected`，说明字典错了。用诊断节点遍历所有字典：

```bash
ros2 run calib_realman diagnose_charuco --ros-args \
    -p camera_topic:=/camera_l/color/image_raw
```
输出会告诉你哪个字典匹配：
```
=== BEST MATCH: DICT_5X5_100 with 24 markers ===
```
然后把 `config/charuco_board.yaml` 里的 `dictionary` 字段改成对应值。

### 手动调用服务测试

不用键盘也能测：
```bash
ros2 service call /data_collector/capture std_srvs/srv/Trigger
ros2 service call /data_collector/save_dataset std_srvs/srv/Trigger
```

### 常见问题

- **`Service capture not available`** — data_collector 没启动 / 启动失败；先 `ros2 node list` 和 `ros2 service list` 确认
- **`Failed to connect to arm ...`** — 检查机械臂 IP 能否 ping 通、SDK 是否在同一 Python 环境
- **`No ArUco markers detected`** — 字典错，用诊断工具
- **`ArUco detected but ChArUco corners < 6`** — 板子行列 (`squares_x/y`) 错或者部分遮挡
- **手眼标定误差大** — 采集时位姿覆盖度不够，建议包含大角度旋转和不同距离

## 文件结构

```
calib_realman/
├── arm_driver_node.py         # 睿尔曼机械臂驱动
├── data_collector_node.py     # 数据采集
├── calibration_node.py        # 手眼标定计算
├── global_cam_node.py         # 全局相机标定
├── tf_publisher_node.py       # TF 发布
├── diagnose_charuco_node.py   # ArUco 字典诊断
└── utils/
    ├── charuco_detector.py
    ├── hand_eye_solver.py
    └── transform_utils.py

config/
├── calibration_params.yaml    # IP、话题、frame、世界坐标系
└── charuco_board.yaml         # 标定板参数

launch/
├── arm_driver.launch.py
├── collect_data.launch.py
├── calibrate.launch.py
└── publish_tf.launch.py

assets/
└── charuco_board_A4.pdf       # 标定板参考图
```

## 依赖

- ROS2 Humble (Ubuntu 22.04)
- Python SDK: `Robotic_Arm`
- OpenCV: `opencv-contrib-python`
- 其他: `numpy`, `scipy`, `transforms3d`, `pyyaml`


# 误差

● 这两个是手眼标定的自洽性误差，不是真实误差（真实误差需要独立的 ground truth），但足以反映标定质量。

  原理

  手眼标定解的是 AX = XB 方程：
  - A = 机械臂末端在两个位姿之间的运动（从 FK 得到，你认为是准的）
  - B = 相机看到标定板在两个位姿之间的运动（视觉测出来的）
  - X = 相机到末端的变换（要求的结果）

  有了 X 以后，对每一对样本 (i, j) 都能算出 A·X 和 X·B，理想情况下完全相等。实际上总有残差，这就是误差来源：

  ┌───────────────────┬───────────────────────────────────────────────┬──────┐
  │       指标        │                  算的是什么                   │ 单位 │
  ├───────────────────┼───────────────────────────────────────────────┼──────┤
  │ Rotation error    │ A·X 和 X·B 的旋转部分之间的夹角（平均值）     │ 度   │
  ├───────────────────┼───────────────────────────────────────────────┼──────┤
  │ Translation error │ A·X 和 X·B 的平移部分之间的欧氏距离（平均值） │ mm   │
  └───────────────────┴───────────────────────────────────────────────┴──────┘

  代码在 calib_realman/utils/hand_eye_solver.py:compute_reprojection_error。

  正常范围参考

  Rotation error（度）

  ┌───────────┬────────┬────────────────────────────────────────────┐
  │   范围    │  评价  │                    说明                    │
  ├───────────┼────────┼────────────────────────────────────────────┤
  │ < 0.3°    │ 非常好 │ 高精度相机 + 稳定机械臂 + 样本充足         │
  ├───────────┼────────┼────────────────────────────────────────────┤
  │ 0.3° – 1° │ 良好   │ 多数工业场景可用                           │
  ├───────────┼────────┼────────────────────────────────────────────┤
  │ 1° – 2°   │ 可接受 │ 能用，但精细任务（抓取对位、装配）可能不够 │
  ├───────────┼────────┼────────────────────────────────────────────┤
  │ 2° – 5°   │ 偏大   │ 建议排查                                   │
  ├───────────┼────────┼────────────────────────────────────────────┤
  │ > 5°      │ 异常   │ 基本不可用                                 │
  └───────────┴────────┴────────────────────────────────────────────┘

  Translation error（mm）

  ┌────────────┬────────┬─────────────┐
  │    范围    │  评价  │    说明     │
  ├────────────┼────────┼─────────────┤
  │ < 2 mm     │ 非常好 │             │
  ├────────────┼────────┼─────────────┤
  │ 2 – 5 mm   │ 良好   │ 多数场景 OK │
  ├────────────┼────────┼─────────────┤
  │ 5 – 10 mm  │ 可接受 │ 粗抓取可用  │
  ├────────────┼────────┼─────────────┤
  │ 10 – 20 mm │ 偏大   │ 排查        │
  ├────────────┼────────┼─────────────┤
  │ > 20 mm    │ 异常   │ 有问题      │
  └────────────┴────────┴─────────────┘

  ▎ 对 D435 + RM65 这个组合，拍摄距离 30-60cm 的话，正常应落在旋转 < 1°、平移 < 5mm。

  误差偏大的常见原因

  1. 相机内参不准 — /camera_*/camera_info 里的 K、D 是工厂标定的，但有时偏差大。可以用 camera_calibration 包重新做相机内参标定
  2. 样本位姿覆盖不够
    - 只做了平移，没有旋转 → 旋转方向欠定
    - 角度变化 < 30° → 矩阵条件数差，解不稳
    - 建议：每个轴的旋转至少覆盖 ±30°，位置覆盖工作空间不同高度/距离
  3. 样本数太少 — 建议 ≥ 15 个，< 8 个精度明显下降
  4. ChArUco 板检测角点少 — 采集时至少 15 个以上角点，越多越稳。日志里的 Corners: N 可以看
  5. 标定板离相机太远/太近 — D435 建议 30-80cm，太远角点精度差，太近只能看到局部
  6. 标定板本身不平整 — 纸贴曲面或起皱会让 solvePnP 误差变大
  Translation error（mm）

  ┌────────────┬────────┬─────────────┐
  │    范围    │  评价  │    说明     │
  ├────────────┼────────┼─────────────┤
  │ < 2 mm     │ 非常好 │             │
  ├────────────┼────────┼─────────────┤
  │ 2 – 5 mm   │ 良好   │ 多数场景 OK │
  ├────────────┼────────┼─────────────┤
  │ 5 – 10 mm  │ 可接受 │ 粗抓取可用  │
  ├────────────┼────────┼─────────────┤
  │ 10 – 20 mm │ 偏大   │ 排查        │
  ├────────────┼────────┼─────────────┤
  │ > 20 mm    │ 异常   │ 有问题      │
  └────────────┴────────┴─────────────┘

  ▎ 对 D435 + RM65 这个组合，拍摄距离 30-60cm 的话，正常应落在旋转 < 1°、平移 < 5mm。

  误差偏大的常见原因

  1. 相机内参不准 — /camera_*/camera_info 里的 K、D 是工厂标定的，但有时偏差大。可以用 camera_calibration 包重新做相机内参标定
  2. 样本位姿覆盖不够
    - 只做了平移，没有旋转 → 旋转方向欠定
    - 角度变化 < 30° → 矩阵条件数差，解不稳
    - 建议：每个轴的旋转至少覆盖 ±30°，位置覆盖工作空间不同高度/距离
  3. 样本数太少 — 建议 ≥ 15 个，< 8 个精度明显下降
  4. ChArUco 板检测角点少 — 采集时至少 15 个以上角点，越多越稳。日志里的 Corners: N 可以看
  5. 标定板离相机太远/太近 — D435 建议 30-80cm，太远角点精度差，太近只能看到局部
  6. 标定板本身不平整 — 纸贴曲面或起皱会让 solvePnP 误差变大
  7. 机械臂姿态读取与图像不同步 — 采集瞬间机械臂还没停稳。建议每次采集前停 0.5-1 秒
  8. TCP 没校准（如果装了夹爪/工具） — 末端位姿算的是法兰盘还是工具尖？跟你 ee_frame 的定义要一致

  排查顺序

  Rotation > 2° 或 Translation > 10mm
    ↓
  看 dataset.json 里每个样本的 corners_detected，有没有 < 15 的
    ↓
  看样本的 EE 位姿是否覆盖了不同旋转（检查 ee_pose_matrix 的旋转分量）
    ↓
  重新采集：停稳再拍 + 增加旋转覆盖
    ↓
  还不行 → 重做相机内参标定

  如果你现在已经跑出结果，贴一下标定节点的输出（那三行 Rotation error: X deg、Translation error: X mm、样本数），我帮你判断是否需要重采。

  