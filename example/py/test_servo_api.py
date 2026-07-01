#!/usr/bin/env python3
#
# test_servo_api.py
# @brief 伺服状态与速度控制示例程序
# @attention
#   - 确保机械臂供电正常、网络通信正常
#   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#   - 运行模式：示教模式
#   - 伺服状态机: 0=停止 → 1=就绪 → 3=运行（上电后）
#   - set_servo_state(1) 必须先于 set_servo_poweron 调用
#   - set_servo_poweroff 只能在 state=3 时调用，成功后回到 state=1
# @note 运行步骤
#       运行: python3 test_servo_api.py
#

import sys
import time

# 添加 SDK Python 绑定路径
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *


# ============================================================
# 工具函数
# ============================================================

def print_result(msg, ret):
    """打印操作结果"""
    if ret == SUCCESS:
        print("  [成功] " + msg)
    else:
        print("  [失败] " + msg + "，错误码: " + str(ret))


def print_separator(title):
    """打印分隔标题"""
    print("\n========== " + title + " ==========")


def servo_state_text(state):
    """获取伺服状态文本描述"""
    if state == 0:
        return "0=停止"
    elif state == 1:
        return "1=就绪"
    elif state == 2:
        return "2=报警"
    elif state == 3:
        return "3=运行"
    else:
        return "未知"


# ============================================================
# 主函数
# ============================================================

def main():
    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[信息] SDK 版本: " + version)

    # ---- 连接控制器 ----
    print_separator("连接控制器")
    sock = connect_robot("192.168.1.13", "6001")
    if sock < 0:
        print("[ERROR] 连接控制器失败")
        return -1
    print("[信息] 连接成功 (socket = " + str(sock) + ")")

    # ============================================================
    # 1. 查询初始伺服状态
    # ============================================================
    print_separator("查询初始伺服状态")

    state = -1
    ret, state = get_servo_state(sock, state)
    if ret == SUCCESS:
        print("  [信息] 初始伺服状态: " + servo_state_text(state))
    else:
        print_result("get_servo_state", ret)

    # ============================================================
    # 2. 切换为示教模式
    # ============================================================
    # 伺服状态操作通常在示教模式下进行（0=示教）
    print_separator("切换示教模式")
    ret = set_current_mode(sock, 0)
    print_result("set_current_mode(0)", ret)
    time.sleep(1)

    # ============================================================
    # 3. 清错处理
    # ============================================================
    # 如果伺服处于报警状态（state=2），需要先清错
    state = -1
    _, state = get_servo_state(sock, state)
    if state == 2:
        print_separator("清错")
        ret = clear_error(sock)
        print_result("clear_error", ret)
        time.sleep(0.3)

    # ============================================================
    # 4. 设置伺服就绪 → 上电 → 验证状态变化
    # ============================================================
    print_separator("伺服上电流程")

    # 4a. 设置伺服就绪（state=1）
    print("[步骤 4a] 设置伺服就绪 (set_servo_state=1)")
    ret = set_servo_state(sock, 1)
    if ret == OPERATION_NOT_ALLOWED:
        print("  [信息] 机械臂已处于就绪或运行状态，无需设置")
    else:
        print_result("set_servo_state(1)", ret)
        if ret != SUCCESS:
            disconnect_robot(sock)
            return -1
    time.sleep(0.5)

    # 验证当前状态
    state = -1
    _, state = get_servo_state(sock, state)
    print("  [信息] 当前伺服状态: " + servo_state_text(state))

    # 4b. 上电使能（state=1 → state=3）
    print("\n[步骤 4b] 上电使能 (set_servo_poweron)")
    ret = set_servo_poweron(sock)
    print_result("set_servo_poweron", ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1
    time.sleep(1)

    # 验证上电成功后的状态应为 3=运行
    state = -1
    ret, state = get_servo_state(sock, state)
    if ret == SUCCESS:
        print("  [信息] 上电后伺服状态: " + servo_state_text(state), end="")
        if state == 3:
            print("（上电成功）")
        print("")

    # ============================================================
    # 5. 速度查询与设置
    # ============================================================
    print_separator("速度查询与设置")

    # 5a. 查询当前速度
    speed = -1
    ret, speed = get_speed(sock, speed)
    if ret == SUCCESS:
        print("  [信息] 当前全局速度: " + str(speed) + "%")
    else:
        print_result("get_speed", ret)

    # 5b. 设置速度并验证
    # set_speed 范围 0<speed≤100，设置后影响后续所有运动的速度上限
    test_speeds = [50, 80, 30]
    for spd in test_speeds:
        print("\n[步骤 5] 设置全局速度: " + str(spd) + "%")
        ret = set_speed(sock, spd)
        print_result("set_speed", ret)
        time.sleep(0.2)

        current_speed = -1
        ret, current_speed = get_speed(sock, current_speed)
        if ret == SUCCESS:
            print("  [信息] 读取确认: 当前速度 = " + str(current_speed) + "%")

    # ============================================================
    # 6. 下电 → 停止 → 验证状态变化
    # ============================================================
    print_separator("伺服下电流程")

    # 6a. 下电（state=3 → state=1）
    print("[步骤 6a] 下电 (set_servo_poweroff)")
    ret = set_servo_poweroff(sock)
    if ret == SUCCESS:
        print("  [成功] set_servo_poweroff")
    elif ret == RECEIVE_FAILED:
        # 下电后网络可能自动断开，-1 视为正常
        print("  [信息] set_servo_poweroff 返回 -1（网络可能已断开）")
    else:
        print_result("set_servo_poweroff", ret)
    time.sleep(0.5)

    # 6b. 验证下电后的伺服状态
    state = -1
    ret, state = get_servo_state(sock, state)
    if ret == SUCCESS:
        print("  [信息] 下电后伺服状态: " + servo_state_text(state))

    # 6c. 设置伺服停止（state=1 → state=0）
    print("\n[步骤 6c] 设置伺服停止 (set_servo_state=0)")
    ret = set_servo_state(sock, 0)
    if ret == OPERATION_NOT_ALLOWED:
        print("  [信息] 当前状态不允许直接设为停止")
    else:
        print_result("set_servo_state(0)", ret)
    time.sleep(0.3)

    # 最终状态确认
    state = -1
    ret, state = get_servo_state(sock, state)
    if ret == SUCCESS:
        print("  [信息] 最终伺服状态: " + servo_state_text(state))

    # ============================================================
    # 7. 断开连接
    # ============================================================
    print_separator("断开连接")
    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] 伺服状态示例程序运行完毕")
    return 0


if __name__ == "__main__":
    exit(main())
