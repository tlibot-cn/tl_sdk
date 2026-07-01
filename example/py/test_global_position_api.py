# /**
#  * test_global_position_api.py
#  * @brief 全局路点管理接口（包括设置全局路点，查询全局路点）
#  * @attention 确保机械臂供电正常、网络通信正常
#  * @note 运行步骤
#  *       运行: python3 test_global_position_api.py
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
    elif state == 1:
        # 就绪态：直接上电
        set_servo_poweron(socket_fd)
    elif state == 2:
        # 报警态：清错 → 就绪 → 上电
        clear_error(socket_fd)
        set_servo_state(socket_fd, 1)
        time.sleep(0.5)
        set_servo_poweron(socket_fd)
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

    # ----设置全局路点----
    print("[INFO] **********设置全局路点**********")
    posName = "GP_test0"
    posInfo = [0, 1, 2, 3, 4, 5, 6]
    ret = set_global_position(socket_fd, posName, posInfo)
    if ret != SUCCESS:
        print("[ERROR] 设置全局路点失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 设置全局路点成功")
    print()

    # ----查询全局路点----
    print("[INFO] **********查询全局路点**********")
    queriedPos = VectorDouble()
    ret = get_global_position(socket_fd, posName, queriedPos)
    if ret != SUCCESS:
        print("[ERROR] 查询全局路点失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 查询全局路点成功，位置为：", end="")
    for pos in queriedPos:
        print(str(pos) + " ", end="")
    print()

    # 使能下电
    disable_servo(socket_fd)

    # 断开控制器连接
    print("[INFO] 断开控制器连接...")
    ret = disconnect_robot(socket_fd)
    if ret != SUCCESS:
        print("[ERROR] 断开连接失败，程序退出!")
        sys.exit(-1)
    print("[INFO] 断开连接成功")
