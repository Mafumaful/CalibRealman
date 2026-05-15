"""相机位姿实时监视节点

以 ChArUco 标定板坐标系为原点，实时计算、终端打印并可视化相机位姿。

典型用途：
  - 标定前确认 ChArUco 检测质量
  - 采集数据时引导机械臂到合适位置（看 Z 距离 / 旋转角）
  - 标定后验证 pose 是否合理

用法：
    ros2 run calib_realman cam_pose_monitor --ros-args \\
        -p camera_topic:=/camera_l/color/image_raw \\
        -p camera_info_topic:=/camera_l/color/camera_info
"""

import collections
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from .utils.charuco_detector import CharucoDetector
from .utils.transform_utils import rvec_tvec_to_matrix, invert_transform


# ── 颜色常量（BGR）─────────────────────────────────────────────────────────
class _C:
    BG        = (30,  30,  30)
    GRID      = (55,  55,  55)
    TRAIL_DIM = (40, 120,  40)
    TRAIL_HI  = (60, 220, 100)
    CAM_FILL  = (0,  200, 255)
    CAM_LINE  = (255, 255, 255)
    BOARD     = (180, 180, 255)
    AXIS_X    = (70,  70, 220)   # red-ish (BGR)
    AXIS_Y    = (70, 200,  70)   # green
    AXIS_Z    = (220,  70,  70)  # blue
    TEXT      = (220, 220, 220)
    TEXT_DIM  = (130, 130, 130)
    OK        = (60,  210,  60)
    WARN      = (30, 160, 255)
    DIVIDER   = (70,  70,  70)


# ── 视图辅助类 ─────────────────────────────────────────────────────────────

class _View2D:
    """把世界坐标（m）映射到像素 ROI 内，并提供基础绘图方法。"""

    def __init__(self, canvas: np.ndarray, roi: tuple,
                 center_w: tuple, half_span: float):
        """
        Args:
            canvas:    全局画布
            roi:       (x, y, w, h) 在画布中的区域
            center_w:  世界坐标中心 (cx, cy)
            half_span: 视野半径（米）
        """
        self.canvas = canvas
        self.rx, self.ry, self.rw, self.rh = roi
        self.cx, self.cy = center_w
        self.half_span = max(half_span, 0.05)

    def to_px(self, wx: float, wy: float):
        margin_x = self.half_span * (self.rw - 40) / self.rw
        margin_y = self.half_span * (self.rh - 40) / self.rh
        px = int(self.rx + self.rw / 2 + (wx - self.cx) / self.half_span * (self.rw / 2 - 20))
        py = int(self.ry + self.rh / 2 - (wy - self.cy) / self.half_span * (self.rh / 2 - 20))
        return px, py

    def in_roi(self, px, py) -> bool:
        return (self.rx <= px < self.rx + self.rw and
                self.ry <= py < self.ry + self.rh)

    def draw_grid(self, step: float = 0.1):
        lo = -self.half_span + self.cx
        hi =  self.half_span + self.cx
        v = lo
        while v <= hi + 1e-9:
            p0 = self.to_px(v, lo - self.cy + self.cy)
            p1 = self.to_px(v, hi - self.cy + self.cy)
            cv2.line(self.canvas, p0, p1, _C.GRID, 1)
            p0 = self.to_px(lo - self.cx + self.cx, v - self.cy + self.cy)
            p1 = self.to_px(hi - self.cx + self.cx, v - self.cy + self.cy)
            cv2.line(self.canvas, p0, p1, _C.GRID, 1)
            v += step

    def draw_grid_auto(self):
        """自动选择网格间距。"""
        span = self.half_span * 2
        raw_step = span / 6
        # 取最近的 0.05 / 0.1 / 0.2 / 0.5 整数倍
        for nice in (0.02, 0.05, 0.1, 0.2, 0.5, 1.0):
            if raw_step <= nice:
                step = nice
                break
        else:
            step = round(raw_step, 1)
        self.draw_grid(step)

    def draw_axes(self):
        """在原点画坐标轴。"""
        o = self.to_px(0, 0)
        ax = self.to_px(self.half_span * 0.15, 0)
        ay = self.to_px(0, self.half_span * 0.15)
        cv2.arrowedLine(self.canvas, o, ax, _C.AXIS_X, 2, tipLength=0.2)
        cv2.arrowedLine(self.canvas, o, ay, _C.AXIS_Y, 2, tipLength=0.2)

    def draw_board(self, half_w: float, half_h: float):
        """在 z=0 处绘制标定板轮廓（以第一个轴为宽，第二轴可忽略显示为一条线）。"""
        # 在 2D 投影里 board 仅是一段线（沿横轴方向）
        p0 = self.to_px(-half_w, 0)
        p1 = self.to_px(+half_w, 0)
        cv2.line(self.canvas, p0, p1, _C.BOARD, 3)
        cv2.putText(self.canvas, 'board', (p0[0], p0[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, _C.BOARD, 1, cv2.LINE_AA)

    def draw_trail(self, trail):
        n = len(trail)
        for k in range(n - 1):
            alpha = (k + 1) / n
            color = tuple(int(_C.TRAIL_DIM[c] + alpha * (_C.TRAIL_HI[c] - _C.TRAIL_DIM[c]))
                          for c in range(3))
            p1 = self.to_px(trail[k][0],     trail[k][1])
            p2 = self.to_px(trail[k + 1][0], trail[k + 1][1])
            if self.in_roi(*p1) or self.in_roi(*p2):
                cv2.line(self.canvas, p1, p2, color, 1, cv2.LINE_AA)

    def draw_camera(self, wx, wy, label: str = ''):
        p = self.to_px(wx, wy)
        if not self.in_roi(*p):
            return
        cv2.circle(self.canvas, p, 7, _C.CAM_FILL, -1)
        cv2.circle(self.canvas, p, 7, _C.CAM_LINE, 1)
        if label:
            cv2.putText(self.canvas, label, (p[0] + 10, p[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, _C.CAM_FILL, 1, cv2.LINE_AA)

    def draw_title_labels(self, title: str, x_label: str, y_label: str):
        cv2.putText(self.canvas, title,
                    (self.rx + 6, self.ry + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, _C.TEXT, 1, cv2.LINE_AA)
        cv2.putText(self.canvas, f'-> {x_label}',
                    (self.rx + self.rw - 80, self.ry + self.rh - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, _C.TEXT_DIM, 1)
        cv2.putText(self.canvas, f'^ {y_label}',
                    (self.rx + 4, self.ry + self.rh - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, _C.TEXT_DIM, 1)

    def draw_scale(self):
        """在右下角标注当前视野范围。"""
        label = f'+/-{self.half_span * 1000:.0f}mm'
        cv2.putText(self.canvas, label,
                    (self.rx + self.rw - 70, self.ry + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, _C.TEXT_DIM, 1)


# ── 主节点 ────────────────────────────────────────────────────────────────

class CamPoseMonitorNode(Node):
    """以标定板为原点实时显示相机位姿。"""

    CAM_VIEW_W = 640
    CAM_VIEW_H = 480
    TRAJ_W     = 420
    TRAJ_H     = 480

    def __init__(self):
        super().__init__('cam_pose_monitor')

        # ── 参数声明 ──────────────────────────────────────────────────────
        self.declare_parameter('camera_topic',      '/camera_l/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_l/color/camera_info')
        self.declare_parameter('squares_x',      5)
        self.declare_parameter('squares_y',      7)
        self.declare_parameter('square_length',  0.04)
        self.declare_parameter('marker_length',  0.03)
        self.declare_parameter('dictionary',     'DICT_5X5_100')
        self.declare_parameter('trail_len',      300)   # 最多保留多少帧轨迹
        self.declare_parameter('print_hz',       5.0)   # 终端打印频率
        self.declare_parameter('window_scale',   1.0)   # 整体窗口缩放

        camera_topic      = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        sq_len            = self.get_parameter('square_length').value
        sq_x              = self.get_parameter('squares_x').value
        sq_y              = self.get_parameter('squares_y').value
        self.board_half_w = sq_len * sq_x / 2.0   # 标定板半宽（X 方向）
        self.board_half_h = sq_len * sq_y / 2.0   # 标定板半高（Y 方向）
        self.trail_len    = self.get_parameter('trail_len').value
        self.print_hz     = self.get_parameter('print_hz').value
        self.win_scale    = self.get_parameter('window_scale').value
        self.axis_len     = sq_len * 2.0           # 绘制坐标轴长度

        # ── ChArUco 检测器 ────────────────────────────────────────────────
        self.charuco = CharucoDetector(
            squares_x=sq_x,
            squares_y=sq_y,
            square_length=sq_len,
            marker_length=self.get_parameter('marker_length').value,
            dictionary_name=self.get_parameter('dictionary').value,
        )

        # ── 状态 ──────────────────────────────────────────────────────────
        self.bridge        = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs   = None
        # 轨迹缓冲：每项为 np.array([x_world, z_world]) 或 [y_world, z_world]
        self.trail_xz = collections.deque(maxlen=self.trail_len)  # X-Z 俯视
        self.trail_yz = collections.deque(maxlen=self.trail_len)  # Y-Z 侧视
        self._last_print   = 0.0
        self._frame_cnt    = 0
        self._detect_cnt   = 0

        # ── 订阅 ─────────────────────────────────────────────────────────
        self.create_subscription(Image,      camera_topic,      self._image_cb, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._info_cb,  10)

        self.get_logger().info(
            f'[CamPoseMonitor] Subscribing to:\n'
            f'  image : {camera_topic}\n'
            f'  info  : {camera_info_topic}')

    # ── 回调 ─────────────────────────────────────────────────────────────

    def _info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info('[CamPoseMonitor] Camera intrinsics received.')

    def _image_cb(self, msg):
        if self.camera_matrix is None:
            return

        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self._frame_cnt += 1

        rvec, tvec, corners, ids, ok = self.charuco.detect_and_estimate(
            image, self.camera_matrix, self.dist_coeffs)

        pos_in_board = None
        rot_in_board = None

        if ok:
            self._detect_cnt += 1
            # solvePnP 给出 T_board_in_cam；取逆得到 T_cam_in_board
            T_b2c = rvec_tvec_to_matrix(rvec, tvec)
            T_c2b = invert_transform(T_b2c)
            pos_in_board = T_c2b[:3, 3]          # [x, y, z] (m)，board 为原点
            rot_in_board = T_c2b[:3, :3]

            # OpenCV ChArUco 板坐标系：X→右，Y↓下，Z 朝板内（远离相机）
            # 相机在板前方时 pos_in_board[2] < 0，取负值得到正的"深度"
            depth = -pos_in_board[2]
            self.trail_xz.append(np.array([pos_in_board[0], depth]))
            self.trail_yz.append(np.array([pos_in_board[1], depth]))

            self._maybe_print(pos_in_board, rot_in_board, len(corners))

        # ── 合并画面 ──────────────────────────────────────────────────────
        left  = self._draw_camera_view(image, rvec, tvec, corners, ids, ok, pos_in_board)
        right = self._draw_trajectory_panel(pos_in_board)
        canvas = np.hstack([left, right])

        if self.win_scale != 1.0:
            h, w = canvas.shape[:2]
            canvas = cv2.resize(canvas,
                                (int(w * self.win_scale), int(h * self.win_scale)))

        cv2.imshow('CamPoseMonitor  [Q to quit]', canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    # ── 终端打印 ─────────────────────────────────────────────────────────

    def _maybe_print(self, pos, rot, n_corners):
        now = time.monotonic()
        if self.print_hz > 0 and (now - self._last_print) < 1.0 / self.print_hz:
            return
        self._last_print = now

        x, y, z   = pos * 1000          # 转 mm；Z<0 表示相机在板前方（板Z朝内）
        depth_mm   = -z                  # 正深度 = -Z
        dist       = float(np.linalg.norm(pos)) * 1000
        rpy        = Rotation.from_matrix(rot).as_euler('xyz', degrees=True)
        roll, pitch, yaw = rpy
        det_pct    = 100.0 * self._detect_cnt / max(self._frame_cnt, 1)

        print(
            f'\r[CamInBoard]  '
            f'X={x:+7.1f}  Y={y:+7.1f}  Z={z:+7.1f} mm  '
            f'depth={depth_mm:6.1f} mm  |  '
            f'roll={roll:+6.1f}  pitch={pitch:+6.1f}  yaw={yaw:+6.1f} deg  '
            f'corners={n_corners:2d}  det={det_pct:.0f}%   ',
            end='', flush=True)

    # ── 左侧：相机视图 ───────────────────────────────────────────────────

    def _draw_camera_view(self, image, rvec, tvec, corners, ids,
                          ok: bool, pos) -> np.ndarray:
        # 等比缩放到固定高度
        h, w = image.shape[:2]
        scale = self.CAM_VIEW_H / h
        vis = cv2.resize(image, (int(w * scale), self.CAM_VIEW_H))
        K_scaled = self.camera_matrix.copy()
        K_scaled[:2] *= scale

        if ok:
            # 缩放 corners
            corners_s = [c * scale for c in corners]
            cv2.aruco.drawDetectedCornersCharuco(vis, np.array(corners_s), ids)
            cv2.drawFrameAxes(vis, K_scaled, self.dist_coeffs,
                              rvec, tvec, self.axis_len)

        # ── 半透明信息栏 ──────────────────────────────────────────────────
        overlay = vis.copy()
        box_h = 170 if ok else 60
        cv2.rectangle(overlay, (0, 0), (295, box_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, vis, 0.45, 0, vis)

        y_txt = 18
        def put(text, color, scale=0.52):
            nonlocal y_txt
            cv2.putText(vis, text, (8, y_txt),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
            y_txt += 26

        put('Camera in Board Frame', _C.TEXT, 0.54)
        if ok:
            x_mm, y_mm, z_mm = pos * 1000
            dist_mm = float(np.linalg.norm(pos)) * 1000
            dist_color = _C.OK if 300 <= dist_mm <= 1200 else _C.WARN
            put(f'X = {x_mm:+7.1f} mm', _C.AXIS_X)
            put(f'Y = {y_mm:+7.1f} mm', _C.AXIS_Y)
            put(f'Z = {z_mm:+7.1f} mm  (board-in, cam<0)', _C.AXIS_Z)
            put(f'dist = {dist_mm:.1f} mm', dist_color)
            rpy = Rotation.from_matrix(
                invert_transform(rvec_tvec_to_matrix(rvec, tvec))[:3, :3]
            ).as_euler('xyz', degrees=True)
            put(f'RPY {rpy[0]:+.1f} {rpy[1]:+.1f} {rpy[2]:+.1f} deg', _C.TEXT, 0.46)
            put(f'corners = {len(corners)}', _C.TEXT, 0.44)
        else:
            put('Board NOT detected', _C.WARN)

        # 右下角：检测率
        det_pct = 100.0 * self._detect_cnt / max(self._frame_cnt, 1)
        det_color = _C.OK if det_pct >= 80 else _C.WARN
        cv2.putText(vis,
                    f'det {det_pct:.0f}%  frame {self._frame_cnt}',
                    (vis.shape[1] - 145, vis.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, det_color, 1)

        # 如果视图宽度不足 CAM_VIEW_W，用黑色填充到固定宽度
        if vis.shape[1] < self.CAM_VIEW_W:
            pad = np.zeros((self.CAM_VIEW_H, self.CAM_VIEW_W - vis.shape[1], 3),
                           dtype=np.uint8)
            vis = np.hstack([vis, pad])
        else:
            vis = vis[:, :self.CAM_VIEW_W]
        return vis

    # ── 右侧：轨迹面板 ───────────────────────────────────────────────────

    def _draw_trajectory_panel(self, pos_now) -> np.ndarray:
        panel = np.full((self.TRAJ_H, self.TRAJ_W, 3), _C.BG, dtype=np.uint8)
        half_h = self.TRAJ_H // 2

        # 计算自适应视野范围
        xz_span = self._auto_span(self.trail_xz, default_c=(0.0, 0.6), default_half=0.4)
        yz_span = self._auto_span(self.trail_yz, default_c=(0.0, 0.6), default_half=0.4)

        # ── 俯视图：X-Z（上半）─────────────────────────────────────────────
        v_xz = _View2D(panel,
                       roi=(0, 0, self.TRAJ_W, half_h),
                       center_w=xz_span['center'],
                       half_span=xz_span['half'])
        v_xz.draw_grid_auto()
        v_xz.draw_axes()
        v_xz.draw_board(self.board_half_w, 0)  # board 沿 X 轴展开，Z=0
        v_xz.draw_trail(self.trail_xz)
        if pos_now is not None:
            depth = -pos_now[2]   # board Z 朝内，相机在负Z侧，取负得正深度
            label = f'{pos_now[0]*1000:+.0f}, {depth*1000:.0f}mm'
            v_xz.draw_camera(pos_now[0], depth, label)
        v_xz.draw_title_labels('Top View  (X - depth)', 'X', 'depth(-Z)')
        v_xz.draw_scale()

        # ── 侧视图：Y-Z（下半）─────────────────────────────────────────────
        v_yz = _View2D(panel,
                       roi=(0, half_h, self.TRAJ_W, self.TRAJ_H - half_h),
                       center_w=yz_span['center'],
                       half_span=yz_span['half'])
        v_yz.draw_grid_auto()
        v_yz.draw_axes()
        v_yz.draw_board(self.board_half_h, 0)  # board 沿 Y 轴展开，Z=0
        v_yz.draw_trail(self.trail_yz)
        if pos_now is not None:
            depth = -pos_now[2]
            label = f'{pos_now[1]*1000:+.0f}, {depth*1000:.0f}mm'
            v_yz.draw_camera(pos_now[1], depth, label)
        v_yz.draw_title_labels('Side View (Y - depth)', 'Y', 'depth(-Z)')
        v_yz.draw_scale()

        # 分隔线
        cv2.line(panel, (0, half_h), (self.TRAJ_W, half_h), _C.DIVIDER, 1)

        # 图例
        self._draw_legend(panel)
        return panel

    def _auto_span(self, trail, default_c, default_half) -> dict:
        """根据轨迹计算合适的视图中心和半径。"""
        if len(trail) < 2:
            return {'center': default_c, 'half': default_half}
        pts = np.array(trail)
        cx = float(np.mean(pts[:, 0]))
        cy = float(np.mean(pts[:, 1]))
        half = max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])) * 0.65 + 0.08
        half = max(half, 0.1)
        return {'center': (cx, cy), 'half': half}

    def _draw_legend(self, panel):
        items = [
            (_C.BOARD,    '-  board'),
            (_C.TRAIL_HI, '-  trail'),
            (_C.CAM_FILL, '*  camera now'),
        ]
        x0, y0 = self.TRAJ_W - 110, self.TRAJ_H - 10 - len(items) * 16
        for i, (color, label) in enumerate(items):
            cv2.putText(panel, label, (x0, y0 + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, color, 1, cv2.LINE_AA)


# ── 入口 ─────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CamPoseMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()   # 终端换行，清除 \r 行
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
