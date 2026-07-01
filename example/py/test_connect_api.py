#!/usr/bin/env python3
#
# test_connect_api.py
# @brief 机械臂连接相关接口（包括连接控制器，断开控制器连接）
# @attention 确保机械臂供电正常、网络通信正常
# @note 运行步骤
#       运行: python3 test_connect_api.py
#

import sys
import time

# 添加 SDK Python 绑定路径
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *


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

    print("[INFO] 延时2s")
    time.sleep(2)

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
