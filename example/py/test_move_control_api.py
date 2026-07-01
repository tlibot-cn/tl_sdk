# /**
#  * test_move_control_api.py
#  * @brief 运动控制相关接口（包括MoveJ、MoveL）
#  * @attention
#  *   - 运行模式：示教模式
#  *   - 开始前需: connect_robot → set_servo_state(1) → set_servo_poweron
#  *   - 退出前需: set_servo_poweroff → disconnect_robot
#  *   - 确保机械臂供电正常、网络通信正常
#  * @note 运行步骤
#  *       运行: python3 test_move_control_api.py
#  */

import sys
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *
import time


# ============================================================
# 工具函数
# ============================================================

def print_result(msg, ret):
    """输出操作结果

    @param msg 操作描述
    @param ret 返回码（Result枚举值）
    """
    if ret == SUCCESS:
        print("  [成功] " + msg)
    else:
        print("  [失败] " + msg + "，错误码: " + str(ret))


def print_separator(title):
    """输出分隔标题"""
    print("\n========== " + title + " ==========")


def enable_servo(socket_fd):
    """使能上电（处理各种伺服状态）

    @param socket_fd 6001端口 socket
    """
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 0:
        set_servo_state(socket_fd, 1)
        set_servo_poweron(socket_fd)
    elif state == 1:
        set_servo_poweron(socket_fd)
    elif state == 2:
        clear_error(socket_fd)
        set_servo_state(socket_fd, 1)
        set_servo_poweron(socket_fd)
    elif state == 3:
        print("[INFO] 机械臂已在使能上电状态")

    _, state = get_servo_state(socket_fd, state)
    if state == 3:
        print("[INFO] 机械臂使能上电成功（servo_state = " + str(state) + "）")
    else:
        print("[INFO] 机械臂使能上电失败（servo_state = " + str(state) + "）")


def disable_servo(socket_fd):
    """使能下电

    @param socket_fd 6001端口 socket
    """
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 1:
        print("[INFO] 机械臂已使能下电")
    elif state == 3:
        set_servo_poweroff(socket_fd)
        _, state = get_servo_state(socket_fd, state)
        print("[INFO] 机械臂使能下电成功（servo_state = " + str(state) + "）")
        return

    print("[INFO] 机械臂使能下电失败（servo_state = " + str(state) + "）")


# ============================================================
# 主函数
# ============================================================

def main():
    # ---- 定义IP地址和端口 ----
    ip = "192.168.1.13"
    port = "6001"

    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[INFO] SDK 版本: " + version)

    # ---- 连接控制器 ----
    print("[INFO] 连接控制器...")
    socket_fd = connect_robot(ip, port)
    if socket_fd < 0:
        print("[ERROR] 连接失败，程序退出!")
        return -1
    print("[INFO] 连接成功 (socket=" + str(socket_fd) + ")")

    # ---- 切换为示教模式 ----
    print("--- 切换示教模式 ---")
    ret = set_current_mode(socket_fd, 0)
    if ret != SUCCESS:
        print("[ERROR] 切换示教模式失败，程序退出!")
        return -1
    print("[INFO] 已切换为示教模式")
    print()

    # ---- 使能上电 ----
    enable_servo(socket_fd)

    # ---- MoveJ运动控制 ----
    print("[INFO] **********MoveJ运动控制**********")
    cmd = MoveCmd()
    cmd.targetPosType = PosType_PType
    cmd.targetPosName = ""
    cmd.coord = 0              # 关节坐标系
    cmd.velocity = 40          # 关节速度百分比 (1,100]
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0                 # 平滑参数 [0,5]
    cmd.targetPosValue = VectorDouble([0.0] * 14)
    ret = robot_movej(socket_fd, cmd)
    if ret != SUCCESS:
        print("[ERROR] MoveJ运动控制失败，程序退出!")
        return -1
    print("[INFO] MoveJ运动控制成功")
    print()

    time.sleep(10.0)

    # ---- MoveL运动控制 ----
    print("[INFO] **********MoveL运动控制**********")
    cmd.targetPosName = ""
    cmd.coord = 1              # 直角坐标系
    cmd.targetPosValue = VectorDouble([280, 0.0, 380, 3.14, 0.0, 0.0])
    ret = robot_movel(socket_fd, cmd)
    if ret != SUCCESS:
        print("[ERROR] MoveL运动控制失败，程序退出!")
        return -1
    print("[INFO] MoveL运动控制成功")
    print()

    time.sleep(10.0)

    # ---- 使能下电 ----
    disable_servo(socket_fd)

    # ---- 断开控制器连接 ----
    print("[INFO] 断开控制器连接...")
    ret = disconnect_robot(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 断开连接失败，程序退出!")
        return -1
    print("[INFO] 断开连接成功")

    return 0


if __name__ == "__main__":
    main()
