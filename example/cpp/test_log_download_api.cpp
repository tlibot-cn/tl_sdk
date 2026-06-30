/**
 * test_log_download_api.cpp
 * @brief 日志下载相关接口
 * @attention 确保机械臂供电正常、网络通信正常，且日志保存路径有效
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_log_download_api
 */

#include <iostream>
#include "cpp/interface/tl_api.h"

int main() {
    // 定义IP地址和端口
    std::string ip = "192.168.1.13";
    std::string port = "6001";

    // 定义日志文件路径
    int count = 10;
    std::string log_path = "./log";
    
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

    // 下载日志
    std::cout << "[INFO] 开始下载日志..." << std::endl;
    auto ret = log_download_by_quantity(socket_fd, count, log_path);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 日志下载失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 日志下载完成" << std::endl;

    // 断开控制器连接
    std::cout << "[INFO] 断开控制器连接..." << std::endl;
    ret = disconnect_robot(socket_fd);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 断开连接失败，程序退出!"<< std::endl;
        return -1;
    }
    std::cout << "[INFO] 断开连接成功" << std::endl;

    return 0;
}
