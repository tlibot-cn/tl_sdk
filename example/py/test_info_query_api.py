#!/usr/bin/env python3
#
# test_info_query_api.py
# @brief 相关信息查询接口（包括当前位置、运行状态、关节角度、DH参数等）
# @attention 确保机械臂供电正常、网络通信正常
# @note 运行步骤
#       运行: python3 test_info_query_api.py
#

import sys
import time

# 添加 SDK Python 绑定路径
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *


def enable_servo(socket_fd):
    """使能上电 —— 先清错，再检查伺服状态，报警则报错退出，最后上电"""
    print("--- 检查伺服状态 ---")

    # 1. 先清除控制器错误和报警，再下电复位确保状态同步
    ret = clear_error(socket_fd)
    time.sleep(0.3)
    set_servo_poweroff(socket_fd)
    time.sleep(0.3)

    # 2. 检查伺服状态
    state = -1
    ret, state = get_servo_state(socket_fd, state)
    if ret != SUCCESS:
        print(f"[ERROR] 获取伺服状态失败: {ret}")
        return
    print(f"  [信息] 当前伺服状态: {state} (0=停止 1=就绪 2=报警 3=运行)")

    if state == 2:
        print("[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试")
        return

    if state == 3:
        print("[INFO] 机械臂已在使能上电状态")
        return

    # 3. 上电流程
    if state == 0:
        print("  [信息] 伺服处于停止状态，设置就绪...")
        ret = set_servo_state(socket_fd, 1)
        if ret != SUCCESS and ret != OPERATION_NOT_ALLOWED:
            print(f"[ERROR] 设置伺服就绪失败: {ret}")
            return
        time.sleep(0.5)

    print("  [信息] 上电使能...")
    ret = set_servo_poweron(socket_fd)
    if ret != SUCCESS:
        print(f"[ERROR] 上电失败: {ret}")
        return
    time.sleep(2)

    # 4. 确认结果
    _, state = get_servo_state(socket_fd, state)
    if state == 3:
        print("[INFO] 机械臂使能上电成功（servo_state = " + str(state) + "）")
    else:
        print("[ERROR] 机械臂使能上电失败（servo_state = " + str(state) + "）")


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

    print("")

    # ----获取机械臂当前坐标----
    print("[INFO] **********获取机械臂当前坐标**********")
    coord = 0
    current_position = VectorDouble()
    ret = get_current_position(socket_fd, coord, current_position)
    if ret != SUCCESS:
        print("[ERROR] 获取机械臂当前坐标失败，程序退出!")
        return -1
    print("[INFO] 机械臂当前坐标: ", end="")
    for pos in current_position:
        print(str(pos) + " ", end="")
    print("")
    print("")

    # ----获取机械臂运行状态----
    print("[INFO] **********获取机械臂运行状态**********")
    status = 0
    ret, status = get_robot_running_state(socket_fd, status)
    if ret != SUCCESS:
        print("[ERROR] 获取机械臂运行状态失败，程序退出!")
        return -1
    if status == 0:
        print("[INFO] 机械臂处于停止状态 (status = " + str(status) + ")")
    elif status == 1:
        print("[INFO] 机械臂处于暂停状态 (status = " + str(status) + ")")
    elif status == 2:
        print("[INFO] 机械臂处于运行状态 (status = " + str(status) + ")")
    print("")

    # ----获取关节角度----
    print("[INFO] **********获取关节角度**********")
    axis_num = 7
    joint_position = VectorDouble()
    ret = get_joint_position(socket_fd, axis_num, joint_position)
    if ret != SUCCESS:
        print("[ERROR] 获取关节角度失败，程序退出!")
        return -1
    print("[INFO] 机械臂当前关节角度: ", end="")
    for joint_pos in joint_position:
        print(str(joint_pos) + " ", end="")
    print("")
    print("")

    # ----获取机械臂DH参数----
    print("[INFO] **********获取机械臂DH参数**********")
    dh_params = RobotDHParam()
    ret = get_robot_dh_param(socket_fd, dh_params)
    if ret != SUCCESS:
        print("[ERROR] 获取机械臂DH参数失败，程序退出!")
        return -1
    print("[INFO] 机械臂DH参数: ", end="")
    print("L1=" + str(dh_params.L1) + ", L2=" + str(dh_params.L2) + ", L3=" + str(dh_params.L3)
          + ", L4=" + str(dh_params.L4) + ", L5=" + str(dh_params.L5) + ", L6=" + str(dh_params.L6)
          + ", L7=" + str(dh_params.L7) + ", L8=" + str(dh_params.L8) + ", L9=" + str(dh_params.L9)
          + ", L10=" + str(dh_params.L10) + ", L11=" + str(dh_params.L11) + ", L12=" + str(dh_params.L12)
          + ", L13=" + str(dh_params.L13) + ", L14=" + str(dh_params.L14) + ", L15=" + str(dh_params.L15)
          + ", L16=" + str(dh_params.L16) + ", L17=" + str(dh_params.L17) + ", L18=" + str(dh_params.L18)
          + ", L19=" + str(dh_params.L19) + ", L20=" + str(dh_params.L20)
          + ", Couple_Coe_1_2=" + str(dh_params.Couple_Coe_1_2)
          + ", Couple_Coe_2_3=" + str(dh_params.Couple_Coe_2_3)
          + ", Couple_Coe_3_2=" + str(dh_params.Couple_Coe_3_2)
          + ", Couple_Coe_3_4=" + str(dh_params.Couple_Coe_3_4)
          + ", Couple_Coe_4_5=" + str(dh_params.Couple_Coe_4_5)
          + ", Couple_Coe_4_6=" + str(dh_params.Couple_Coe_4_6)
          + ", Couple_Coe_5_6=" + str(dh_params.Couple_Coe_5_6)
          + ", dynamicLimit_max=" + str(dh_params.dynamicLimit_max)
          + ", dynamicLimit_min=" + str(dh_params.dynamicLimit_min)
          + ", pitch=" + str(dh_params.pitch)
          + ", sliding_lead_value=" + str(dh_params.sliding_lead_value)
          + ", uplift_lead_value=" + str(dh_params.uplift_lead_value)
          + ", spray_distance=" + str(dh_params.spray_distance)
          + ", threeAxisDirection=" + str(dh_params.threeAxisDirection)
          + ", fiveAxisDirection=" + str(dh_params.fiveAxisDirection)
          + ", twoAxisConversionRatio=" + str(dh_params.twoAxisConversionRatio)
          + ", threeAxisConversionRatio=" + str(dh_params.threeAxisConversionRatio)
          + ", amplificationRatio=" + str(dh_params.amplificationRatio)
          + ", conversionratio_x=" + str(dh_params.conversionratio_x)
          + ", conversionratio_y=" + str(dh_params.conversionratio_y)
          + ", conversionratio_z=" + str(dh_params.conversionratio_z)
          + ", conversionratio_J1=" + str(dh_params.conversionratio_J1)
          + ", conversionratio_J2=" + str(dh_params.conversionratio_J2)
          + ", conversionratio_J3=" + str(dh_params.conversionratio_J3)
          + ", upsideDown=" + str(dh_params.upsideDown)
          + ", hanyu.PC=" + str(dh_params.hanyu.PC)
          + ", hanyu.SP[0]=" + str(dh_params.hanyu.SP[0])
          + ", hanyu.SP[1]=" + str(dh_params.hanyu.SP[1])
          + ", hanyu.SP[2]=" + str(dh_params.hanyu.SP[2])
          + ", hanyu.TL[0]=" + str(dh_params.hanyu.TL[0])
          + ", hanyu.TL[1]=" + str(dh_params.hanyu.TL[1])
          + ", hanyu.TL[2]=" + str(dh_params.hanyu.TL[2]))
    print("")

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
