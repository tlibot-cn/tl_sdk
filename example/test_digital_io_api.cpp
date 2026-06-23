/**
 * test_digital_io_api.cpp
 * @brief  模式与 IO 控制接口（包括设置机械臂模式、设置数字 IO 输出、设置数字 IO 输出、获取所有 IO 输入状态）
 * @attention 确保机械臂供电正常、网络通信正常
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_digital_io_api
 */

#include <iostream>
#include "cpp/interface/tl_api.h"

void enable_servo(const int socket_fd)
{
  int state = -1;
  get_servo_state(socket_fd, state);
  switch (state)
  {
    case 0:
      set_servo_state(socket_fd, 1);
      set_servo_poweron(socket_fd);
      break;
    case 1:
      set_servo_poweron(socket_fd);
      break;
    case 2:
      clear_error(socket_fd);
      set_servo_state(socket_fd, 1);
      set_servo_poweron(socket_fd);
      break;
    case 3:
      std::cout << "[INFO] 机械臂已在使能上电状态" << std::endl;
    default:
      break;
  }

  get_servo_state(socket_fd, state);
  switch (state)
  {
    case 3:
        std::cout << "[INFO] 机械臂使能上电成功（servo_state = " << state << "）" << std::endl;
        break;
    default:
        std::cout << "[INFO] 机械臂使能上电失败（servo_state = " << state << "）" << std::endl;
        break;
  }
}

void disable_servo(const int socket_fd)
{
  int state = -1;
  get_servo_state(socket_fd, state);
  switch (state)
  {
    case 1:
        std::cout << "[INFO] 机械臂已使能下电" << std::endl;
        break;
    case 3:
        set_servo_poweroff(socket_fd);
        get_servo_state(socket_fd, state);
        std::cout << "[INFO] 机械臂使能下电成功（servo_state = " << state << "）" << std::endl;
        return;
    default:
        break;
  }
  std::cout << "[INFO] 机械臂使能下电失败（servo_state = " << state << "）" << std::endl;
}

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

    // 使能上电
    enable_servo(socket_fd);

    // ----设置机器人模式----
    std::cout << "[INFO] **********设置机器人模式**********" << std::endl;
    int mode = 1;
    auto ret = set_current_mode(socket_fd, mode);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 设置机器人模式失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 设置机器人模式成功" << std::endl;
    std::cout << std::endl;

    // ----设置数字 IO 输出----
    std::cout << "[INFO] **********设置数字 IO 输出**********" << std::endl;
    int io_port = 1;
    int value = 0;
    ret = set_digital_output(socket_fd, io_port, value);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 设置数字 IO 输出失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 设置数字 IO 输出成功" << std::endl;
    std::cout << std::endl;

    // ----查询数字 IO 输出----
    std::cout << "[INFO] **********查询数字 IO 输出**********" << std::endl;
    std::vector<int> outputValues;
    ret = get_digital_output(socket_fd, outputValues);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 查询数字 IO 输出失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 查询数字 IO 输出成功，状态为：";
    for (const auto& state : outputValues) {
        std::cout << state << " ";
    }
    std::cout << std::endl;

    // ----获取所有 IO 输入状态----
    std::cout << "[INFO] **********获取所有 IO 输入状态**********" << std::endl;
    std::vector<int> inputValues;
    ret = get_digital_input(socket_fd, inputValues);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 查询数字 IO 输入失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 查询数字 IO 输入成功，状态为：";
    for (const auto& state : inputValues) {
        std::cout << state << " ";
    }
    std::cout << std::endl;

    // 使能下电
    disable_servo(socket_fd);

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
