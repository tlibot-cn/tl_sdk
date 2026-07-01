# /**
#  * test_modbus_api.py
#  * @brief Modbus 主站通信示例程序
#  * @attention
#  *   - 确保控制器网络通信正常
#  *   - Modbus 从站设备需正确连接并配置
#  *   - 默认 TCP 从站 IP 为 192.168.1.13，端口 503
#  *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
#  * @note 运行步骤
#  *       运行: python3 test_modbus_api.py
#  */

import sys
sys.path.append("../../lib/linux/x86/python")
from tl_interface import *
import time


# ============================================================
# 工具函数
# ============================================================

def print_result(msg, ret):
    """@brief 打印操作结果"""
    if ret == SUCCESS:
        print("  [成功] %s" % msg)
    else:
        print("  [失败] %s，错误码: %d" % (msg, ret))


def print_separator(title):
    """@brief 打印分隔标题"""
    print("\n========== %s ==========" % title)


# ============================================================
# 主函数
# ============================================================

def main():
    # ---- 获取 SDK 版本 ----
    version = get_library_version()
    print("[信息] SDK 版本: %s" % version)

    # ---- 连接控制器 ----
    print_separator("连接控制器")
    sock = connect_robot("192.168.1.13", "6001")
    if sock < 0:
        print("[ERROR] 连接控制器失败")
        return -1
    print("[信息] 连接成功 (socket = %d)" % sock)

    # ============================================================
    # 1. 配置 Modbus 主站参数
    # ============================================================
    # 支持两种模式:
    #   TCP — 通过网络连接远程 Modbus 从站
    #   RTU — 通过串口连接本地 Modbus 从站
    print_separator("配置 Modbus 主站参数")

    param = ModbusMasterParameter()
    param.type = "TCP"                    # 主站类型: TCP / RTU
    param.startAddress = False             # false=起始地址1, true=起始地址0
    param.TCP.IP = "192.168.1.13"          # 从站 IP 地址
    param.TCP.port = 503                   # 从站端口

    # id 为配方编号，范围 [0, 8]（最多保存 9 个配方）
    ret = modbus_set_master_parameter(sock, 0, param)
    print_result("modbus_set_master_parameter (id=0, TCP)", ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1

    # ============================================================
    # 2. 打开 Modbus 主站
    # ============================================================
    print_separator("打开 Modbus 主站")

    ret = modbus_open_master(sock, 0)
    print_result("modbus_open_master (id=0)", ret)
    if ret != SUCCESS:
        disconnect_robot(sock)
        return -1
    time.sleep(0.5)

    # ============================================================
    # 3. 写保持寄存器 (Modbus 功能码 10H)
    # ============================================================
    # 向从站地址 0 开始连续写入 3 个寄存器
    print_separator("写保持寄存器")

    write_data = [100, 200, 300]
    start_addr = 0

    ret = modbus_write_multiple_holding_registers(sock, 0, start_addr, write_data)
    print_result("modbus_write_multiple_holding_registers", ret)
    if ret == SUCCESS:
        print("  写入地址: %d，数据: %d, %d, %d" % (start_addr, write_data[0], write_data[1], write_data[2]))

    # ============================================================
    # 4. 读保持寄存器 (Modbus 功能码 03H)
    # ============================================================
    # 从之前写入的地址读取 3 个寄存器，验证写入结果
    print_separator("读保持寄存器")

    read_data = VectorInt()
    start_addr = 0
    quantity = 3

    ret = modbus_read_holding_registers(sock, 0, start_addr, quantity, read_data)
    print_result("modbus_read_holding_registers", ret)
    if ret == SUCCESS:
        print("  读取地址: %d，数量: %d" % (start_addr, quantity))
        print("  读取数据: ", end="")
        for i in range(len(read_data)):
            print("%d" % read_data[i], end="")
            if i < len(read_data) - 1:
                print(", ", end="")
        print()

        # 验证读写一致性
        if len(read_data) >= 3:
            match = (read_data[0] == 100 and read_data[1] == 200 and read_data[2] == 300)
            print("  读写一致性验证: %s" % ("通过" if match else "失败"))

    # ============================================================
    # 5. 断开连接
    # ============================================================
    print_separator("断开连接")
    disconnect_robot(sock)
    print("[信息] 已断开连接")

    print("\n[信息] Modbus 示例程序运行完毕")
    return 0


if __name__ == "__main__":
    main()
