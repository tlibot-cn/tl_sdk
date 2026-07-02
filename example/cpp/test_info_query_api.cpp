/**
 * test_info_query_api.cpp
 * @brief 相关信息查询接口（包括）
 * @attention 确保机械臂供电正常、网络通信正常，
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_info_query
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

    std::cout << std::endl;

    // ----获取机械臂当前坐标----
    std::cout << "[INFO] **********获取机械臂当前坐标**********" << std::endl;
    int coord = 0;
    std::vector<double> current_position;
    auto ret = get_current_position(socket_fd, coord, current_position);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 获取机械臂当前坐标失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 机械臂当前坐标: ";
    for (const auto& pos : current_position) {
        std::cout << pos << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;

    // ----获取机械臂运行状态----
    std::cout << "[INFO] **********获取机械臂运行状态**********" << std::endl;
    int status = 0; 
    ret = get_robot_running_state(socket_fd, status);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 获取机械臂运行状态失败，程序退出!" << std::endl;
        return -1;
    }
    switch (status)
    {
        case 0:
            std::cout << "[INFO] 机械臂处于停止状态 (status = " << status << ")" << std::endl;
            break;
        case 1:
            std::cout << "[INFO] 机械臂处于暂停状态 (status = " << status << ")" << std::endl;
            break;
        case 2:
            std::cout << "[INFO] 机械臂处于运行状态 (status = " << status << ")" << std::endl;
            break;
        default:
            break;
    }
    std::cout << std::endl;
    
    // ----获取关节角度----
    std::cout << "[INFO] **********获取关节角度**********" << std::endl;
    int axisNum = 7;
    std::vector<double> joint_position;
    ret = get_joint_position(socket_fd, axisNum, joint_position);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 获取关节角度失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 机械臂当前关节角度: ";
    for (const auto& joint_pos : joint_position) {
        std::cout << joint_pos << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;


    // 获取机械臂DH参数
    std::cout << "[INFO] **********获取机械臂DH参数**********" << std::endl;
    RobotDHParam dh_params;
    ret = get_robot_dh_param(socket_fd, dh_params);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 获取机械臂DH参数失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 机械臂DH参数: ";
    std::cout << "L1=" << dh_params.L1 << ", L2=" << dh_params.L2 << ", L3=" << dh_params.L3 
              << ", L4=" << dh_params.L4 << ", L5=" << dh_params.L5 << ", L6=" << dh_params.L6 
              << ", L7=" << dh_params.L7 << ", L8=" << dh_params.L8 << ", L9=" << dh_params.L9 
              << ", L10=" << dh_params.L10 << ", L11=" << dh_params.L11 << ", L12=" << dh_params.L12 
              << ", L13=" << dh_params.L13 << ", L14=" << dh_params.L14 << ", L15=" << dh_params.L15 
              << ", L16=" << dh_params.L16 << ", L17=" << dh_params.L17 << ", L18=" << dh_params.L18 
              << ", L19=" << dh_params.L19 << ", L20=" << dh_params.L20 
              << ", Couple_Coe_1_2=" << dh_params.Couple_Coe_1_2 
              << ", Couple_Coe_2_3=" << dh_params.Couple_Coe_2_3 
              << ", Couple_Coe_3_2=" << dh_params.Couple_Coe_3_2 
              << ", Couple_Coe_3_4=" << dh_params.Couple_Coe_3_4 
              << ", Couple_Coe_4_5=" << dh_params.Couple_Coe_4_5 
              << ", Couple_Coe_4_6=" << dh_params.Couple_Coe_4_6 
              << ", Couple_Coe_5_6=" << dh_params.Couple_Coe_5_6 
              << ", dynamicLimit_max=" << dh_params.dynamicLimit_max 
              << ", dynamicLimit_min=" << dh_params.dynamicLimit_min 
              << ", pitch=" << dh_params.pitch 
              << ", sliding_lead_value=" << dh_params.sliding_lead_value 
              << ", uplift_lead_value=" << dh_params.uplift_lead_value 
              << ", spray_distance=" << dh_params.spray_distance 
              << ", threeAxisDirection=" << dh_params.threeAxisDirection 
              << ", fiveAxisDirection=" << dh_params.fiveAxisDirection 
              << ", twoAxisConversionRatio=" << dh_params.twoAxisConversionRatio 
              << ", threeAxisConversionRatio=" << dh_params.threeAxisConversionRatio 
              << ", amplificationRatio=" << dh_params.amplificationRatio 
              << ", conversionratio_x=" << dh_params.conversionratio_x 
              << ", conversionratio_y=" << dh_params.conversionratio_y 
              << ", conversionratio_z=" << dh_params.conversionratio_z 
              << ", conversionratio_J1=" << dh_params.conversionratio_J1 
              << ", conversionratio_J2=" << dh_params.conversionratio_J2 
              << ", conversionratio_J3=" << dh_params.conversionratio_J3 
              << ", upsideDown=" << dh_params.upsideDown 
              << ", hanyu.PC=" << dh_params.hanyu.PC 
              << ", hanyu.SP[0]=" << dh_params.hanyu.SP[0] 
              << ", hanyu.SP[1]=" << dh_params.hanyu.SP[1] 
              << ", hanyu.SP[2]=" << dh_params.hanyu.SP[2] 
              << ", hanyu.TL[0]=" << dh_params.hanyu.TL[0] 
              << ", hanyu.TL[1]=" << dh_params.hanyu.TL[1] 
              << ", hanyu.TL[2]=" << dh_params.hanyu.TL[2] 
              << std::endl;
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
