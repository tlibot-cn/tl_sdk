/**
 * test_connect_api.cpp
 * @brief 机械臂连接相关接口（包括连接控制器，断开控制器连接）
 * @attention 确保机械臂供电正常、网络通信正常
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_connect_api
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "cpp/interface/tl_api.h"

int main() {
    // 定义IP地址和端口
    std::string ip = "192.168.1.13";
    std::string port = "6001";

    // 获取 SDK 版本
    std::string version = get_library_version();
    std::cout << "[INFO] SDK 版本: " << version << std::endl;

    // 连接控制器
    std::cout << "[INFO] 连接控制器..." << std::endl;
    auto socket_fd = connect_robot(ip, port);
    if (socket_fd < 0) {
        std::cerr << "[ERROR] 连接失败，程序退出!"<< std::endl;
        return -1;
    }
    std::cout << "[INFO] 连接成功 (socket=" << socket_fd << ")" << std::endl;

    std::cout << "[INFO] 延时2s" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 断开控制器连接
    std::cout << "[INFO] 断开控制器连接..." << std::endl;
    auto ret = disconnect_robot(socket_fd);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 断开连接失败，程序退出!"<< std::endl;
        return -1;
    }
    std::cout << "[INFO] 断开连接成功" << std::endl;

    return 0;
}