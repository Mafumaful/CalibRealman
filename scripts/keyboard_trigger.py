"""键盘触发脚本：按空格键触发数据采集，按s保存数据集，按q退出。

必须在真实终端 (TTY) 中运行，例如:
    ros2 run calib_realman keyboard_trigger
不能通过 launch 的 ExecuteProcess 启动（无 TTY）。
"""

import sys

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(f'[OK] {result.message}')
            else:
                self.get_logger().warn(f'[FAIL] {result.message}')
        else:
            self.get_logger().error(f'Service call {name} failed.')


def get_key():
    """读取单个按键（Linux终端，需要TTY）。"""
    import termios
    import tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main(args=None):
    # TTY 检查
    if not sys.stdin.isatty():
        print('[ERROR] keyboard_trigger requires a real TTY.', file=sys.stderr)
        print('Please run it directly in a terminal:', file=sys.stderr)
        print('    ros2 run calib_realman keyboard_trigger', file=sys.stderr)
        sys.exit(1)

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
