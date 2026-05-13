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
from tf2_ros import TransformBroadcaster
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

        self.declare_parameter('rate', 30.0)  # Hz
        self.declare_parameter('log_level', 3)  # 0=debug 1=info 2=warn 3=error
        # 睿尔曼欧拉角约定: 常见取值 'xyz'(外旋) / 'XYZ'(内旋) / 'zyx' / 'ZYX'
        # 睿尔曼官方文档通常是 ZYX 外旋（RPY: roll-pitch-yaw）
        self.declare_parameter('euler_convention', 'xyz')

        ip = self.get_parameter(f'{prefix}.ip').value
        port = self.get_parameter(f'{prefix}.port').value
        self.base_frame = self.get_parameter(f'{prefix}.base_frame').value
        self.ee_frame = self.get_parameter(f'{prefix}.ee_frame').value
        rate = self.get_parameter('rate').value
        log_level = self.get_parameter('log_level').value
        self.euler_convention = self.get_parameter('euler_convention').value

        self._print_loaded_params(ip, port, rate)

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

        # 定时轮询
        self.timer = self.create_timer(1.0 / rate, self._poll_callback)
        self.get_logger().info(
            f'Polling arm state at {rate} Hz, '
            f'publishing TF {self.base_frame} -> {self.ee_frame}')

    def _print_loaded_params(self, ip, port, rate):
        lines = [
            '=' * 60,
            f'  Arm Driver Loaded Parameters [{self.arm_name}]',
            '=' * 60,
            f'  arm_name    : {self.arm_name}',
            f'  ip          : {ip}',
            f'  port        : {port}',
            f'  base_frame  : {self.base_frame}',
            f'  ee_frame    : {self.ee_frame}',
            f'  rate (Hz)   : {rate}',
            '=' * 60,
        ]
        for line in lines:
            self.get_logger().info(line)

    def _poll_callback(self):
        """轮询机械臂状态并发布TF + JointState。"""
        ret, state = self.arm.rm_get_current_arm_state()
        if ret != 0:
            self.get_logger().warn(
                f'{C.YELLOW}[POLL FAIL]{C.RESET} '
                f'{C.YELLOW}rm_get_current_arm_state failed, ret={ret}{C.RESET}',
                throttle_duration_sec=2.0)
            return

        now = self.get_clock().now().to_msg()

        # 位姿: [x, y, z, rx, ry, rz] (m + rad)
        pose = state['pose']
        x, y, z, rx, ry, rz = pose
        # 按配置的欧拉角约定转四元数
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
        self.joint_pub.publish(js)

    def destroy_node(self):
        try:
            if hasattr(self, 'arm') and self.arm is not None:
                self.arm.rm_delete_robot_arm()
                self.get_logger().info(
                    f'{C.CYAN}[DISCONNECT]{C.RESET} '
                    f'{C.CYAN}Disconnected from [{self.arm_name}].{C.RESET}')
        except Exception as e:
            self.get_logger().warn(
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
