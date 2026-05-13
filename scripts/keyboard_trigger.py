"""键盘触发脚本：按空格键触发数据采集，按s保存数据集，按q退出。"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class KeyboardTrigger(Node):
    def __init__(self):
        super().__init__('keyboard_trigger')
        self.capture_client = self.create_client(
            Trigger, '/data_collector/capture')
        self.save_client = self.create_client(
            Trigger, '/data_collector/save_dataset')

        self.get_logger().info(
            'Keyboard trigger ready.\n'
            '  [SPACE] - Capture sample\n'
            '  [s]     - Save dataset\n'
            '  [q]     - Quit')

    def call_service(self, client, name):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service {name} not available.')
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(f'[OK] {result.message}')
            else:
                self.get_logger().warn(f'[FAIL] {result.message}')
        else:
            self.get_logger().error(f'Service call {name} failed.')


def get_key():
    """读取单个按键（Linux终端）。"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTrigger()

    try:
        while True:
            key = get_key()
            if key == ' ':
                node.call_service(node.capture_client, 'capture')
            elif key == 's':
                node.call_service(node.save_client, 'save_dataset')
            elif key == 'q' or key == '\x03':  # q or Ctrl+C
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
