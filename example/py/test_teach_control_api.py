# /**
#  * test_teach_control_api.py
#  * @brief 示教控制相关接口
#  * @attention
#  *   - 运行模式: 示教模式
#  *   - 开始前需: connect_robot → set_servo_state(1) → set_servo_poweron
#  *   - 退出前需: set_servo_poweroff → disconnect_robot
#  *   - 确保机械臂供电正常、网络通信正常
#  * @note 运行步骤
#  *       运行: python3 test_teach_control_api.py
#  */

import sys
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *
import time


# ----工具函数----
def print_result(msg, ret):
    """打印操作结果"""
    if ret == SUCCESS:
        print("  [成功] " + msg)
    else:
        print("  [失败] " + msg + "，错误码: " + str(ret))


def print_separator(title):
    """打印分隔标题"""
    print("\n========== " + title + " ==========")


def enable_servo(socket_fd):
    """使能上电 —— 根据当前伺服状态自动完成清错、就绪、上电操作"""
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 0:
        # 停止态：先就绪再上电
        set_servo_state(socket_fd, 1)
        time.sleep(0.5)
        set_servo_poweron(socket_fd)
        time.sleep(2)
    elif state == 1:
        # 就绪态：直接上电
        set_servo_poweron(socket_fd)
        time.sleep(2)
    elif state == 2:
        # 报警态：清错 → 就绪 → 上电
        clear_error(socket_fd)
        set_servo_state(socket_fd, 1)
        time.sleep(0.5)
        set_servo_poweron(socket_fd)
        time.sleep(2)
    elif state == 3:
        print("[INFO] 机械臂已在使能上电状态")
    else:
        pass

    # 确认上电结果
    _, state = get_servo_state(socket_fd, state)
    if state == 3:
        print("[INFO] 机械臂使能上电成功（servo_state = " + str(state) + "）")
    else:
        print("[INFO] 机械臂使能上电失败（servo_state = " + str(state) + "）")


def disable_servo(socket_fd):
    """使能下电 —— 安全关闭伺服"""
    state = -1
    _, state = get_servo_state(socket_fd, state)
    if state == 1:
        print("[INFO] 机械臂已使能下电")
    elif state == 3:
        set_servo_poweroff(socket_fd)
        _, state = get_servo_state(socket_fd, state)
        print("[INFO] 机械臂使能下电成功（servo_state = " + str(state) + "）")
        return
    else:
        pass
    print("[INFO] 机械臂使能下电失败（servo_state = " + str(state) + "）")


# ============================================================
# 主程序
# ============================================================
if __name__ == "__main__":
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
        sys.exit(-1)
    print("[INFO] 连接成功 (socket=" + str(socket_fd) + ")")

    # 使能上电
    enable_servo(socket_fd)

    # ----切换为示教模式----
    # 示教点动需要在示教模式下执行（0=示教）
    print("--- 切换示教模式 ---")
    ret = set_current_mode(socket_fd, 0)
    if ret != SUCCESS:
        print("[ERROR] 切换示教模式失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 已切换为示教模式")

    # ----示教点动（注意安全）----
    # ⚠️  robot_start_jogging 会直接驱动机器人运动，运行前确保安全区域无人
    print("[INFO] **********示教点动**********")
    axis = 1
    direction = True
    ret = robot_start_jogging(socket_fd, axis, direction)
    if ret != SUCCESS:
        print("[ERROR] 示教点动失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 示教点动成功")

    time.sleep(5)

    # 使能下电
    disable_servo(socket_fd)

    # 断开控制器连接
    print("[INFO] 断开控制器连接...")
    ret = disconnect_robot(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 断开连接失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 断开连接成功")
