#!/usr/bin/env python3
#
# test_error_zero_api.py
# @brief 错误处理接口（清除错误）
# @attention 确保机械臂供电正常、网络通信正常
# @warning
#   set_axis_zero_position（设置关节零位）属于高级标定操作，
#   错误设置会导致机器人定位偏差甚至机械碰撞，默认已注释。
#   如需使用，请仔细阅读控制器手册并在专业人员指导下操作。
# @note 运行步骤
#       运行: python3 test_error_zero_api.py
#

import sys
import time

# 添加 SDK Python 绑定路径
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *


def enable_servo(socket_fd):
    """使能伺服上电"""
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
    """使能伺服下电"""
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


def main():
    # 定义IP地址和端口
    ip = "192.168.1.13"
    port = "6001"

    # 获取 SDK 版本
    version = get_library_version()
    print("[INFO] SDK 版本: " + version)

    # 连接控制器
    print("[INFO] 连接控制器...")
    socket_fd = connect_robot(ip, port)
    if socket_fd < 0:
        print("[ERROR] 连接失败，程序退出!")
        return -1
    print("[INFO] 连接成功 (socket=" + str(socket_fd) + ")")

    # 使能上电
    enable_servo(socket_fd)

    # ----清除错误----
    print("[INFO] **********清除错误**********")
    ret = clear_error(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 清除错误失败，程序退出!")
        return -1
    print("[INFO] 清除错误成功")
    print("")

    # ----设置指定关节零位（危险操作，默认注释）----
    # ⚠️  危险: set_axis_zero_position 会修改机器人关节零位标定，
    #    错误的值会导致所有后续运动出现定位偏差，严重时可能造成机械碰撞。
    #    仅在专业指导下、确认当前零位确需校正时使用。
    #    取消下面代码的注释前，请确认 axis 值正确且已理解全部后果。
    #
    # print("[INFO] **********设置指定关节零位**********")
    # axis = 1
    # ret = set_axis_zero_position(socket_fd, axis)
    # if ret != SUCCESS:
    #     print("[ERROR] 设置指定关节零位失败，程序退出!")
    #     return -1
    # print("[INFO] 设置指定关节零位成功")
    # print("")

    # 使能下电
    disable_servo(socket_fd)

    # 断开控制器连接
    print("[INFO] 断开控制器连接...")
    ret = disconnect_robot(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 断开连接失败，程序退出!")
        return -1
    print("[INFO] 断开连接成功")

    return 0


if __name__ == "__main__":
    exit(main())
