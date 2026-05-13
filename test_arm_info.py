"""睿尔曼机械臂连接测试脚本（独立运行，不依赖ROS）。

用法:
    python3 test_arm_info.py [IP] [PORT]
例:
    python3 test_arm_info.py 192.168.30.123 8080
"""

import sys

try:
    from Robotic_Arm.rm_robot_interface import RoboticArm
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
except ImportError:
    print('[ERROR] Robotic_Arm SDK 未安装，请运行: pip install Robotic_Arm')
    sys.exit(1)


# ANSI 颜色
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
CYAN = '\033[96m'
BOLD = '\033[1m'
RESET = '\033[0m'


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else '192.168.30.123'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080

    print(f'\n{CYAN}{BOLD}=== 睿尔曼机械臂连接测试 ==={RESET}')
    print(f'{CYAN}Target: {ip}:{port}{RESET}\n')

    # 1. 初始化
    arm = RoboticArm(rm_thread_mode_e.RM_SINGLE_MODE_E)

    # 2. 连接
    print(f'{YELLOW}[1/4] 正在连接...{RESET}')
    handle = arm.rm_create_robot_arm(ip, port, level=3)
    if handle.id == -1:
        print(f'{RED}{BOLD}[FAIL]{RESET} {RED}无法连接到 {ip}:{port}'
              f'{RESET}')
        print(f'{YELLOW}排查建议:{RESET}')
        print(f'{YELLOW}  1. ping {ip} 是否通{RESET}')
        print(f'{YELLOW}  2. 机械臂是否上电{RESET}')
        print(f'{YELLOW}  3. 端口是否正确（默认 8080）{RESET}')
        sys.exit(1)
    print(f'{GREEN}[OK]{RESET} {GREEN}连接成功，handle.id = {handle.id}{RESET}\n')

    # 3. 机械臂基本信息（型号、DOF）
    print(f'{YELLOW}[2/4] 读取机械臂基本信息 (rm_get_robot_info)...{RESET}')
    ret, info = arm.rm_get_robot_info()
    if ret == 0:
        print(f'{GREEN}[OK]{RESET}')
        print(f'  {BOLD}型号 (arm_model)              :{RESET} {info.get("arm_model")}')
        print(f'  {BOLD}自由度 (arm_dof)              :{RESET} {info.get("arm_dof")}')
        print(f'  {BOLD}力控类型 (force_type)         :{RESET} {info.get("force_type")}')
        print(f'  {BOLD}控制器版本 (controller_version):{RESET} {info.get("robot_controller_version")}')
    else:
        print(f'{RED}[FAIL]{RESET} ret={ret}')

    # 4. 软件版本信息（含产品型号字符串）
    print(f'\n{YELLOW}[3/4] 读取软件版本信息 (rm_get_arm_software_info)...{RESET}')
    ret, sw = arm.rm_get_arm_software_info()
    if ret == 0:
        print(f'{GREEN}[OK]{RESET}')
        # 不同字段在不同控制器版本可能不一样，全打印出来
        for k, v in sw.items():
            print(f'  {BOLD}{k:30s}:{RESET} {v}')
    else:
        print(f'{RED}[FAIL]{RESET} ret={ret}')

    # 5. 当前位姿（验证通信正常）
    print(f'\n{YELLOW}[4/4] 读取当前状态 (rm_get_current_arm_state)...{RESET}')
    ret, state = arm.rm_get_current_arm_state()
    if ret == 0:
        print(f'{GREEN}[OK]{RESET}')
        pose = state['pose']
        joints = state['joint']
        print(f'  {BOLD}末端位置 (m)        :{RESET} '
              f'x={pose[0]:+.4f}, y={pose[1]:+.4f}, z={pose[2]:+.4f}')
        print(f'  {BOLD}末端欧拉角 (rad)    :{RESET} '
              f'rx={pose[3]:+.4f}, ry={pose[4]:+.4f}, rz={pose[5]:+.4f}')
        # 顺便算下度数方便人眼看
        import math
        rxd, ryd, rzd = (math.degrees(a) for a in pose[3:])
        print(f'  {BOLD}末端欧拉角 (deg)    :{RESET} '
              f'rx={rxd:+.2f}, ry={ryd:+.2f}, rz={rzd:+.2f}')
        print(f'  {BOLD}关节角度 (deg)      :{RESET} '
              f'{[round(j, 2) for j in joints]}')
    else:
        print(f'{RED}[FAIL]{RESET} ret={ret}')

    # 6. 断开
    arm.rm_delete_robot_arm()
    print(f'\n{CYAN}{BOLD}=== 测试完成，已断开连接 ==={RESET}\n')


if __name__ == '__main__':
    main()
