# /**
#  * test_digital_io_api.py
#  * @brief  模式与 IO 控制接口（包括设置机械臂模式、设置数字 IO 输出、查询数字 IO 输出、获取所有 IO 输入状态）
#  * @attention 确保机械臂供电正常、网络通信正常
#  * @note 运行步骤
#  *       运行: python3 test_digital_io_api.py
#  */

import sys
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *
import time


def enable_servo(socket_fd):
    """@brief 使能上电（根据当前伺服状态执行不同流程）"""
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
        print("[INFO] 机械臂使能上电成功（servo_state = %d）" % state)
    else:
        print("[INFO] 机械臂使能上电失败（servo_state = %d）" % state)


def disable_servo(socket_fd):
    """@brief 使能下电"""
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 1:
        print("[INFO] 机械臂已使能下电")
        return
    elif state == 3:
        set_servo_poweroff(socket_fd)
        _, state = get_servo_state(socket_fd, state)
        print("[INFO] 机械臂使能下电成功（servo_state = %d）" % state)
        return
    print("[INFO] 机械臂使能下电失败（servo_state = %d）" % state)


def main():
    # 定义IP地址和端口
    ip = "192.168.1.13"
    port = "6001"

    # 获取 SDK 版本
    version = get_library_version()
    print("[INFO] SDK 版本: %s" % version)

    # 连接控制器
    print("[INFO] 连接控制器...")
    socket_fd = connect_robot(ip, port)
    if socket_fd < 0:
        print("[ERROR] 连接失败，程序退出!")
        return -1
    print("[INFO] 连接成功 (socket=%d)" % socket_fd)

    # 使能上电
    enable_servo(socket_fd)

    # ----设置机器人模式----
    print("[INFO] **********设置机器人模式**********")
    mode = 1
    ret = set_current_mode(socket_fd, mode)
    if ret != SUCCESS:
        print("[ERROR] 设置机器人模式失败，程序退出!")
        return -1
    print("[INFO] 设置机器人模式成功")
    print()

    # ----设置数字 IO 输出----
    print("[INFO] **********设置数字 IO 输出**********")
    io_port = 1
    value = 0
    ret = set_digital_output(socket_fd, io_port, value)
    if ret != SUCCESS:
        print("[ERROR] 设置数字 IO 输出失败，程序退出!")
        return -1
    print("[INFO] 设置数字 IO 输出成功")
    print()

    # ----查询数字 IO 输出----
    print("[INFO] **********查询数字 IO 输出**********")
    outputValues = VectorInt()
    ret = get_digital_output(socket_fd, outputValues)
    if ret != SUCCESS:
        print("[ERROR] 查询数字 IO 输出失败，程序退出!")
        return -1
    print("[INFO] 查询数字 IO 输出成功，状态为：", end="")
    for state in outputValues:
        print("%d " % state, end="")
    print()

    # ----获取所有 IO 输入状态----
    print("[INFO] **********获取所有 IO 输入状态**********")
    inputValues = VectorInt()
    ret = get_digital_input(socket_fd, inputValues)
    if ret != SUCCESS:
        print("[ERROR] 查询数字 IO 输入失败，程序退出!")
        return -1
    print("[INFO] 查询数字 IO 输入成功，状态为：", end="")
    for state in inputValues:
        print("%d " % state, end="")
    print()

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
    main()
