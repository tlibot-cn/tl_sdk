/**
 * test_error_zero_api.cpp
 * @brief 错误处理接口（清除错误）
 * @attention 确保机械臂供电正常、网络通信正常
 * @warning
 *   set_axis_zero_position（设置关节零位）属于高级标定操作，
 *   错误设置会导致机器人定位偏差甚至机械碰撞，默认已注释。
 *   如需使用，请仔细阅读控制器手册并在专业人员指导下操作。
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_error_zero_api
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

    // ----清除错误----
    std::cout << "[INFO] **********清除错误**********" << std::endl;
    auto ret = clear_error(socket_fd);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 清除错误失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 清除错误成功" << std::endl;
    std::cout << std::endl;

    // ----设置指定关节零位（危险操作，默认注释）----
    // ⚠️  危险: set_axis_zero_position 会修改机器人关节零位标定，
    //    错误的值会导致所有后续运动出现定位偏差，严重时可能造成机械碰撞。
    //    仅在专业指导下、确认当前零位确需校正时使用。
    //    取消下面代码的注释前，请确认 axis 值正确且已理解全部后果。
    //
    // std::cout << "[INFO] **********设置指定关节零位**********" << std::endl;
    // int axis = 1;
    // ret = set_axis_zero_position(socket_fd, axis);
    // if (ret != Result::SUCCESS) {
    //     std::cerr << "[ERROR] 设置指定关节零位失败，程序退出!" << std::endl;
    //     return -1;
    // }
    // std::cout << "[INFO] 设置指定关节零位成功" << std::endl;
    // std::cout << std::endl;

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
