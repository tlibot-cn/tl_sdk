/**
 * test_modbus_api.cpp
 * @brief Modbus 主站通信示例程序
 * @attention
 *   - 确保控制器网络通信正常
 *   - Modbus 从站设备需正确连接并配置
 *   - 默认 TCP 从站 IP 为 192.168.1.13，端口 503
 *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_modbus
 */

#include "cpp/interface/tl_api.h"
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

// ============================================================
// 工具函数
// ============================================================

void print_result(const std::string& msg, int ret)
{
    if (ret == Result::SUCCESS) {
        std::cout << "  [成功] " << msg << std::endl;
    } else {
        std::cout << "  [失败] " << msg << "，错误码: " << ret << std::endl;
    }
}

void print_separator(const std::string& title)
{
    std::cout << "\n========== " << title << " ==========" << std::endl;
}

// ============================================================
// 主函数
// ============================================================

int main()
{
    // ---- 获取 SDK 版本 ----
    {
        std::string version = get_library_version();
        std::cout << "[信息] SDK 版本: " << version << std::endl;
    }

    // ---- 连接控制器 ----
    print_separator("连接控制器");
    SOCKETFD sock = connect_robot("192.168.1.13", "6001");
    if (sock < 0) {
        std::cerr << "[ERROR] 连接控制器失败" << std::endl;
        return -1;
    }
    std::cout << "[信息] 连接成功 (socket = " << sock << ")" << std::endl;

    Result ret;

    // ============================================================
    // 1. 配置 Modbus 主站参数
    // ============================================================
    // 支持两种模式:
    //   TCP — 通过网络连接远程 Modbus 从站
    //   RTU — 通过串口连接本地 Modbus 从站
    print_separator("配置 Modbus 主站参数");

    {
        ModbusMasterParameter param;
        param.type = "TCP";                    // 主站类型: TCP / RTU
        param.startAddress = false;             // false=起始地址1, true=起始地址0
        param.TCP.IP = "192.168.1.13";          // 从站 IP 地址
        param.TCP.port = 503;                   // 从站端口

        // id 为配方编号，范围 [0, 8]（最多保存 9 个配方）
        ret = modbus_set_master_parameter(sock, 0, param);
        print_result("modbus_set_master_parameter (id=0, TCP)", ret);
        if (ret != Result::SUCCESS) {
            disconnect_robot(sock);
            return -1;
        }
    }

    // ============================================================
    // 2. 打开 Modbus 主站
    // ============================================================
    print_separator("打开 Modbus 主站");

    ret = modbus_open_master(sock, 0);
    print_result("modbus_open_master (id=0)", ret);
    if (ret != Result::SUCCESS) {
        disconnect_robot(sock);
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ============================================================
    // 3. 写保持寄存器 (Modbus 功能码 10H)
    // ============================================================
    // 向从站地址 0 开始连续写入 3 个寄存器
    print_separator("写保持寄存器");

    {
        std::vector<int> write_data = {100, 200, 300};
        int start_addr = 0;

        ret = modbus_write_multiple_holding_registers(sock, 0, start_addr, write_data);
        print_result("modbus_write_multiple_holding_registers", ret);
        if (ret == Result::SUCCESS) {
            std::cout << "  写入地址: " << start_addr << "，数据: "
                      << write_data[0] << ", " << write_data[1] << ", " << write_data[2] << std::endl;
        }
    }

    // ============================================================
    // 4. 读保持寄存器 (Modbus 功能码 03H)
    // ============================================================
    // 从之前写入的地址读取 3 个寄存器，验证写入结果
    print_separator("读保持寄存器");

    {
        std::vector<int> read_data;
        int start_addr = 0;
        int quantity = 3;

        ret = modbus_read_holding_registers(sock, 0, start_addr, quantity, read_data);
        print_result("modbus_read_holding_registers", ret);
        if (ret == Result::SUCCESS) {
            std::cout << "  读取地址: " << start_addr << "，数量: " << quantity << std::endl;
            std::cout << "  读取数据: ";
            for (size_t i = 0; i < read_data.size(); ++i) {
                std::cout << read_data[i];
                if (i < read_data.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;

            // 验证读写一致性
            if (read_data.size() >= 3) {
                bool match = (read_data[0] == 100 && read_data[1] == 200 && read_data[2] == 300);
                std::cout << "  读写一致性验证: " << (match ? "通过" : "失败") << std::endl;
            }
        }
    }

    // ============================================================
    // 5. 断开连接
    // ============================================================
    print_separator("断开连接");
    disconnect_robot(sock);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] Modbus 示例程序运行完毕" << std::endl;
    return 0;
}
