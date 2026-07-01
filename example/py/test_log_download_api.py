#!/usr/bin/env python3
#
# test_log_download_api.py
# @brief 日志下载相关接口
# @attention 确保机械臂供电正常、网络通信正常，且日志保存路径有效
# @note 运行步骤
#       运行: python3 test_log_download_api.py
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

    # 定义日志文件路径
    count = 10
    log_path = "./log"

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

    # 下载日志
    print("[INFO] 开始下载日志...")
    ret = log_download_by_quantity(socket_fd, count, log_path)
    if ret != SUCCESS:
        print("[ERROR] 日志下载失败，程序退出!")
        return -1
    print("[INFO] 日志下载完成")

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
