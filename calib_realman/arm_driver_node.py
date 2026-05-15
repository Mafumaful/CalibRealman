"""睿尔曼机械臂驱动节点：通过IP连接机械臂，轮询位姿并发布TF和joint_states。

依赖:
    pip install Robotic_Arm

用法:
    ros2 run calib_realman arm_driver --ros-args \\
        -p arm_name:=arm1 -p ip:=192.168.1.18 -p port:=8080
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

try:
    from Robotic_Arm.rm_robot_interface import RoboticArm
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    SDK_AVAILABLE = True
except ImportError as e:
    SDK_AVAILABLE = False
    _import_err = e


# ANSI 颜色码
class C:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'


class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver')

        # 参数
        self.declare_parameter('arm_name', 'arm1')
        self.arm_name = self.get_parameter('arm_name').value

        # 支持嵌套参数 <arm_name>.ip / .port / .base_frame / .ee_frame
        prefix = self.arm_name
        self.declare_parameter(f'{prefix}.ip', '192.168.1.18')
        self.declare_parameter(f'{prefix}.port', 8080)
        self.declare_parameter(f'{prefix}.base_frame', f'{prefix}_base_link')
        self.declare_parameter(f'{prefix}.ee_frame', f'{prefix}_ee_link')
        self.declare_parameter(
            f'{prefix}.camera_frame',
            'camera_l_link' if self.arm_name == 'arm1' else 'camera_r_link')
        # 相机相对末端: [x,y,z,rx,ry,rz]，平移 m，旋转默认度
        self.declare_parameter(
            f'{prefix}.camera_to_ee', [0.08, 0.0, 0.03, 45.0, 0.0, 0.0])

        self.declare_parameter('rate', 30.0)  # Hz
        self.declare_parameter('publish_camera_tf', True)
        self.declare_parameter('camera_euler_convention', 'xyz')
        self.declare_parameter('camera_rotation_in_degrees', True)
        self.declare_parameter('log_level', 3)  # 0=debug 1=info 2=warn 3=error
        # 睿尔曼欧拉角约定: 'xyz'(外旋) / 'XYZ'(内旋) / 'zyx' / 'ZYX'
        self.declare_parameter('euler_convention', 'xyz')
        # 位置缩放系数: 睿尔曼实际返回 mm，默认 0.001 转为米  --修正： 实际就是返回m
        # 如果你的固件确实返回 m，设为 1.0
        self.declare_parameter('position_scale', 1.0)

        ip = self.get_parameter(f'{prefix}.ip').value
        port = self.get_parameter(f'{prefix}.port').value
        self.base_frame = self.get_parameter(f'{prefix}.base_frame').value
        self.ee_frame = self.get_parameter(f'{prefix}.ee_frame').value
        self.camera_frame = self.get_parameter(f'{prefix}.camera_frame').value
        rate = self.get_parameter('rate').value
        log_level = self.get_parameter('log_level').value
        self.euler_convention = self.get_parameter('euler_convention').value
        self.position_scale = self.get_parameter('position_scale').value
        self.publish_camera_tf = self.get_parameter('publish_camera_tf').value
        camera_euler = self.get_parameter('camera_euler_convention').value
        camera_deg = self.get_parameter('camera_rotation_in_degrees').value
        cam_pose = list(self.get_parameter(f'{prefix}.camera_to_ee').value)
        if len(cam_pose) != 6:
            raise ValueError(
                f'{prefix}.camera_to_ee must be [x,y,z,rx,ry,rz], got {cam_pose}')

        self._camera_translation = [float(cam_pose[0]), float(cam_pose[1]),
                                    float(cam_pose[2])]
        rpy = [float(cam_pose[3]), float(cam_pose[4]), float(cam_pose[5])]
        if camera_deg:
            rpy = [math.radians(v) for v in rpy]
        if camera_euler == 'rotvec':
            self._camera_quat = Rotation.from_rotvec(rpy).as_quat()
        else:
            self._camera_quat = Rotation.from_euler(camera_euler, rpy).as_quat()

        self._print_loaded_params(ip, port, rate, cam_pose, camera_euler, camera_deg)

        if not SDK_AVAILABLE:
            self.get_logger().error(
                f'{C.RED}{C.BOLD}[SDK MISSING]{C.RESET} '
                f'{C.RED}Robotic_Arm SDK not available: {_import_err}{C.RESET}\n'
                f'{C.YELLOW}Install with: pip install Robotic_Arm{C.RESET}')
            raise RuntimeError('Robotic_Arm SDK missing')

        # 连接机械臂
        self.get_logger().info(
            f'{C.YELLOW}{C.BOLD}[CONNECTING]{C.RESET} '
            f'{C.YELLOW}Trying to connect [{self.arm_name}] at {ip}:{port} ...{C.RESET}')

        self.arm = None
        try:
            self.arm = RoboticArm(rm_thread_mode_e.RM_SINGLE_MODE_E)
            handle = self.arm.rm_create_robot_arm(ip, port, level=log_level)
        except Exception as e:
            self.get_logger().error(
                f'{C.RED}{C.BOLD}[CONNECT FAIL]{C.RESET} '
                f'{C.RED}Exception while creating handle: {e}{C.RESET}')
            raise

        if handle.id == -1:
            self.get_logger().error(
                f'{C.RED}{C.BOLD}[CONNECT FAIL]{C.RESET} '
                f'{C.RED}Cannot connect to [{self.arm_name}] at {ip}:{port} '
                f'(handle.id == -1).{C.RESET}\n'
                f'{C.YELLOW}Check: 1) network reachable (ping {ip}); '
                f'2) robot powered on; 3) port {port} correct.{C.RESET}')
            raise RuntimeError(f'Arm connection failed: {ip}:{port}')

        self.get_logger().info(
            f'{C.GREEN}{C.BOLD}[CONNECT OK]{C.RESET} '
            f'{C.GREEN}Connected to [{self.arm_name}] at {ip}:{port}, '
            f'handle.id={handle.id}, DOF={self.arm.arm_dof}{C.RESET}')

        # 发布器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(
            JointState, f'/{self.arm_name}/joint_states', 10)
        # 发布睿尔曼原始位姿 [x,y,z,rx,ry,rz] (m + rad, 未转约定)
        self.raw_pose_pub = self.create_publisher(
            Float64MultiArray, f'/{self.arm_name}/raw_pose', 10)

        # ee_link → camera_link 是固定变换，用 StaticTransformBroadcaster 发布一次即可
        if self.publish_camera_tf:
            self._publish_static_camera_tf()

        # 定时轮询
        self.timer = self.create_timer(1.0 / rate, self._poll_callback)
        tf_desc = f'{self.base_frame} -> {self.ee_frame}'
        if self.publish_camera_tf:
            tf_desc += f'  |  {self.ee_frame} -> {self.camera_frame} (static)'
        self.get_logger().info(
            f'Polling arm state at {rate} Hz, publishing TF: {tf_desc}')

    def _print_loaded_params(self, ip, port, rate, cam_pose, camera_euler, camera_deg):
        rot_unit = 'deg' if camera_deg else 'rad'
        lines = [
            '=' * 60,
            f'  Arm Driver Loaded Parameters [{self.arm_name}]',
            '=' * 60,
            f'  arm_name          : {self.arm_name}',
            f'  ip                : {ip}',
            f'  port              : {port}',
            f'  base_frame        : {self.base_frame}',
            f'  ee_frame          : {self.ee_frame}',
            f'  camera_frame      : {self.camera_frame}',
            f'  publish_camera_tf : {self.publish_camera_tf}',
            f'  camera_to_ee      : {cam_pose} (xyz m, rpy {rot_unit})',
            f'  camera_euler      : {camera_euler}',
            f'  rate (Hz)         : {rate}',
            f'  euler_convention  : {self.euler_convention}',
            f'  position_scale    : {self.position_scale} '
            f'({"mm->m" if self.position_scale == 0.001 else "custom"})',
            '=' * 60,
        ]
        for line in lines:
            self.get_logger().info(line)

    def _publish_static_camera_tf(self):
        """用 StaticTransformBroadcaster 发布一次 ee_link → camera_link。"""
        static_broadcaster = StaticTransformBroadcaster(self)
        cam_t = TransformStamped()
        cam_t.header.stamp = self.get_clock().now().to_msg()
        cam_t.header.frame_id = self.ee_frame
        cam_t.child_frame_id = self.camera_frame
        cam_t.transform.translation.x = self._camera_translation[0]
        cam_t.transform.translation.y = self._camera_translation[1]
        cam_t.transform.translation.z = self._camera_translation[2]
        cam_t.transform.rotation.x = float(self._camera_quat[0])
        cam_t.transform.rotation.y = float(self._camera_quat[1])
        cam_t.transform.rotation.z = float(self._camera_quat[2])
        cam_t.transform.rotation.w = float(self._camera_quat[3])
        static_broadcaster.sendTransform(cam_t)
        self.get_logger().info(
            f'{C.GREEN}[STATIC TF]{C.RESET} '
            f'{C.GREEN}Published {self.ee_frame} -> {self.camera_frame}{C.RESET}')

    def _poll_callback(self):
        """轮询机械臂状态并发布TF + JointState。"""
        ret, state = self.arm.rm_get_current_arm_state()
        if ret != 0:
            self.get_logger().warning(
                f'{C.YELLOW}[POLL FAIL]{C.RESET} '
                f'{C.YELLOW}rm_get_current_arm_state failed, ret={ret}{C.RESET}',
                throttle_duration_sec=2.0)
            return

        now = self.get_clock().now().to_msg()

        # 位姿: [x, y, z, rx, ry, rz] (Realman 实际单位: mm + rad)
        pose = state['pose']
        x_raw, y_raw, z_raw, rx, ry, rz = pose
        # 应用位置缩放 (默认 0.001: mm -> m)
        x = x_raw * self.position_scale
        y = y_raw * self.position_scale
        z = z_raw * self.position_scale

        # print(f'position_scale: {self.position_scale})')

        # print(f'Pose (raw): [{x_raw:.1f}, {y_raw:.1f}, {z_raw:.1f}, '  
        #       f'{rx:.3f}, {ry:.3f}, {rz:.3f}]')
        # print(f'Pose (scaled): [{x:.3f}, {y:.3f}, {z:.3f}, '  
        #       f'{rx:.3f}, {ry:.3f}, {rz:.3f}]')

        # 发布原始位姿（已转米，单位与 board_tvec 一致）
        raw_msg = Float64MultiArray()
        raw_msg.data = [float(x), float(y), float(z),
                        float(rx), float(ry), float(rz)]
        self.raw_pose_pub.publish(raw_msg)

        # 按配置的约定转四元数
        # 特殊值 'rotvec' = 当成旋转向量 (axis-angle) 而非欧拉角
        if self.euler_convention == 'rotvec':
            quat = Rotation.from_rotvec([rx, ry, rz]).as_quat()
        else:
            quat = Rotation.from_euler(
                self.euler_convention, [rx, ry, rz]).as_quat()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.ee_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

        # JointState (弧度)
        joints = state['joint']  # 单位: 度
        js = JointState()
        js.header.stamp = now
        js.name = [f'{self.arm_name}_joint{i+1}' for i in range(len(joints))]
        js.position = [math.radians(float(j)) for j in joints]
        # js.position = [float(j) for j in joints]
        self.joint_pub.publish(js)

    def destroy_node(self):
        try:
            if hasattr(self, 'arm') and self.arm is not None:
                self.arm.rm_delete_robot_arm()
                self.get_logger().info(
                    f'{C.CYAN}[DISCONNECT]{C.RESET} '
                    f'{C.CYAN}Disconnected from [{self.arm_name}].{C.RESET}')
        except Exception as e:
            self.get_logger().warning(
                f'{C.YELLOW}[DISCONNECT WARN]{C.RESET} '
                f'{C.YELLOW}Error during disconnect: {e}{C.RESET}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Fatal: {e}')
        else:
            print(f'Fatal: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
