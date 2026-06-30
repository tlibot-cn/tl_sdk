# /**
#  * test_error_callback_api.py
#  * @brief 错误/警告回调示例程序
#  * @attention
#  *   - 确保控制器网络通信正常
#  *   - 回调函数内不要做耗时操作或阻塞
#  *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#  * @note 运行步骤
#  *       运行: python3 test_error_callback_api.py
#  */

import sys
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *
import time


# ============================================================
# 回调函数（由用户实现，在收到错误/警告时被调用）
# ============================================================

def on_error_or_warning(messageType, message, messageCode):
    """错误/警告消息回调
    @param messageType 消息类型（0=警告 1=错误）
    @param message     消息内容
    @param messageCode 消息码
    """
    type_str = "警告" if messageType == 0 else "错误"
    print("\n>>> [回调触发] " + type_str
          + " | code=" + str(messageCode)
          + " | " + str(message))


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


# ============================================================
# 主程序
# ============================================================
if __name__ == "__main__":
    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[信息] SDK 版本: " + version)

    # ---- 连接控制器 ----
    print_separator("连接控制器")
    sock = connect_robot("192.168.1.13", "6001")
    if sock < 0:
        print("[ERROR] 连接控制器失败")
        sys.exit(-1)
    print("[信息] 连接成功 (socket = " + str(sock) + ")")

    # ============================================================
    # 1. 注册错误/警告回调
    # ============================================================
    print_separator("注册回调")
    ret = set_receive_error_or_warnning_message_callback(sock, on_error_or_warning)
    print_result("set_receive_error_or_warnning_message_callback", ret)
    if ret != SUCCESS:
        print("[ERROR] 注册回调失败")
        disconnect_robot(sock)
        sys.exit(-1)
    print("  [信息] 回调已注册，后续收到错误/警告时会自动触发")

    # ============================================================
    # 2. 逆解不可达点，触发错误回调
    # ============================================================
    # 通过 get_origin_coord_to_target_coord 尝试逆解一个超出工作空间的点，
    # 控制器会返回错误/警告，从而触发注册的回调函数
    print_separator("逆解不可达点")

    print("--- 逆解一个超出工作空间的点，观察回调触发 ---")
    # 设置一个明显不可达的直角坐标点（X 方向超出机械臂最大臂展）
    unreachable_pose = [99999, 0, 0, 0, 0, 0]
    joint_result = VectorDouble()

    # 注意: get_origin_coord_to_target_coord 在 SWIG 4.2.1 x86 绑定中存在 bool& 重载 bug，
    # 此处用 try/except 包装以避免运行时崩溃
    convert_state = False
    try:
        ret = get_origin_coord_to_target_coord(sock, 1, unreachable_pose, 0, joint_result, convert_state)
    except TypeError:
        ret = -5  # EXCEPTION
    print_result("逆解不可达点 (直角→关节)", ret)
    print("  输入直角坐标 [mm, rad]: X=99999, Y=0, Z=0")
    if ret != SUCCESS:
        print("  [信息] 逆解失败，控制器的错误消息已通过回调输出")

    # 等待回调处理完成
    time.sleep(2)

    # ============================================================
    # 3. 断开连接
    # ============================================================
    print_separator("断开连接")
    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] 错误回调示例程序运行完毕")
