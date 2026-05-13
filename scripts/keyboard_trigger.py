"""键盘触发脚本：按空格键触发数据采集，按s保存数据集，按q退出。

必须在真实终端 (TTY) 中运行，例如:
    ros2 run calib_realman keyboard_trigger
不能通过 launch 的 ExecuteProcess 启动（无 TTY）。
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# ANSI 颜色码
class C:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'


CAPTURE_SRV = '/data_collector/capture'
SAVE_SRV = '/data_collector/save_dataset'


class KeyboardTrigger(Node):
    def __init__(self):
        super().__init__('keyboard_trigger')
        self.capture_client = self.create_client(Trigger, CAPTURE_SRV)
        self.save_client = self.create_client(Trigger, SAVE_SRV)

        self.get_logger().info(
            f'{C.CYAN}Keyboard trigger ready.{C.RESET}\n'
            f'  Service [capture]      : {CAPTURE_SRV}\n'
            f'  Service [save_dataset] : {SAVE_SRV}\n'
            f'  {C.BOLD}[SPACE]{C.RESET} - Capture sample\n'
            f'  {C.BOLD}[s]{C.RESET}     - Save dataset\n'
            f'  {C.BOLD}[q]{C.RESET}     - Quit')

    def call_service(self, client, label):
        srv_name = client.srv_name  # 实际服务话题名
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f'{C.RED}{C.BOLD}[SERVICE NOT AVAILABLE]{C.RESET} '
                f'{C.RED}label={label}, topic={srv_name}{C.RESET}\n'
                f'{C.YELLOW}Hint: check `ros2 service list | grep '
                f'{srv_name.split("/")[-1]}` and make sure data_collector '
                f'is running.{C.RESET}')
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    f'{C.GREEN}[OK]{C.RESET} ({srv_name}) {result.message}')
            else:
                self.get_logger().warn(
                    f'{C.YELLOW}[FAIL]{C.RESET} ({srv_name}) {result.message}')
        else:
            self.get_logger().error(
                f'{C.RED}[CALL TIMEOUT]{C.RESET} '
                f'label={label}, topic={srv_name}')


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
