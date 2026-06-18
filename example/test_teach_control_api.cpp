/**
 * test_teach_control_api.cpp
 * @brief 示教控制相关接口
 * @attention 确保机械臂供电正常、网络通信正常
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_teach_control_api
 */

#include <iostream>
#include <thread>
#include <chrono>
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

    // ----示教点动---
    std::cout << "[INFO] **********示教点动**********" << std::endl;
    int axis = 1;
    bool direction = true;
    auto ret = robot_start_jogging(socket_fd, axis, direction);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 示教点动失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 示教点动成功" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(5));

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