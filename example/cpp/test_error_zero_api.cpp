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
#include <thread>
#include <chrono>
#include "cpp/interface/tl_api.h"

void enable_servo(const int socket_fd)
{
  std::cout << "--- 检查伺服状态 ---" << std::endl;

  // 1. 先清除控制器错误和报警，再下电复位确保状态同步
  clear_error(socket_fd);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  set_servo_poweroff(socket_fd);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // 2. 检查伺服状态
  int state = -1;
  auto ret = get_servo_state(socket_fd, state);
  if (ret != Result::SUCCESS) {
    std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
    return;
  }
  std::cout << "  [信息] 当前伺服状态: " << state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

  if (state == 2) {
    std::cerr << "[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试" << std::endl;
    return;
  }

  if (state == 3) {
    std::cout << "[INFO] 机械臂已在使能上电状态" << std::endl;
    return;
  }

  // 3. 上电流程
  if (state == 0) {
    std::cout << "  [信息] 伺服处于停止状态，设置就绪..." << std::endl;
    ret = set_servo_state(socket_fd, 1);
    if (ret != Result::SUCCESS && ret != Result::OPERATION_NOT_ALLOWED) {
      std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << "  [信息] 上电使能..." << std::endl;
  ret = set_servo_poweron(socket_fd);
  if (ret != Result::SUCCESS) {
    std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
    return;
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 4. 确认结果
  get_servo_state(socket_fd, state);
  if (state == 3) {
    std::cout << "[INFO] 机械臂使能上电成功（servo_state = " << state << "）" << std::endl;
  } else {
    std::cerr << "[ERROR] 机械臂使能上电失败（servo_state = " << state << "）" << std::endl;
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
