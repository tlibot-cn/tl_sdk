/**
 * test_job_control_api.cpp
 * @brief 作业控制相关接口（包括作业文件插入MoveJ、MoveL、MoveC，打开作业文件，创建作业文件，获取全部作业文件名，运行作业文件）
 * @attention
 *   - 插入作业之前需要先打开一个作业文件
 *   - 运行一个作业文件需要切换到运行模式
 *   - 确保机械臂供电正常、网络通信正常
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_job_control_api
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

    // ----创建作业文件----
    std::string jobName = "TEST_CPP";
    std::cout << "[INFO] **********创建作业文件**********" << std::endl;
    auto ret = job_create(socket_fd, jobName);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 创建作业文件失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 创建作业文件成功" << std::endl;
    std::cout << std::endl;

    // ----打开作业文件----
    std::cout << "[INFO] **********打开作业文件**********" << std::endl;
    ret = job_open(socket_fd, jobName);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 打开作业文件失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 打开作业文件成功" << std::endl;
    std::cout << std::endl;

    // ----作业文件插入MoveJ----
    std::cout << "[INFO] **********作业文件插入MoveJ**********" << std::endl;
    int line = 1;
    MoveCmd cmd;
    cmd.targetPosType  = PosType::PType;    
    cmd.targetPosName  = "P0001";          
    cmd.coord          = 0;                 
    cmd.velocity       = 40;               
    cmd.acc            = 10;               
    cmd.dec            = 10;              
    cmd.pl             = 0;                 
    cmd.targetPosValue = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ret = job_insert_moveJ(socket_fd, line, cmd);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 作业文件插入MoveJ失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 作业文件插入MoveJ成功" << std::endl;
    std::cout << std::endl;

    // ----作业文件插入MoveL----
    std::cout << "[INFO] **********作业文件插入MoveL**********" << std::endl;
    line = 2;
    cmd.targetPosName  = "P0002";
    cmd.coord          = 1;  
    cmd.targetPosValue = std::vector<double>{280, 0.0, 380, 3.14, 0.0, 0.0};
    ret = job_insert_moveL(socket_fd, line, cmd);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 作业文件插入MoveL失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 作业文件插入MoveL成功" << std::endl;
    std::cout << std::endl;

    // ----作业文件插入MoveC----
    std::cout << "[INFO] **********作业文件插入MoveC**********" << std::endl;
    line = 3;
    cmd.targetPosName  = "P0003";
    cmd.coord          = 2;  
    cmd.targetPosValue = std::vector<double>{270, 0.0, 380, 3.14, 0.0, 0.0};
    ret = job_insert_moveC(socket_fd, line, cmd);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 作业文件插入MoveC失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 作业文件插入MoveC成功" << std::endl;
    std::cout << std::endl;

    // ----作业文件插入MoveC----
    std::cout << "[INFO] **********作业文件插入MoveC**********" << std::endl;
    line = 3;
    cmd.targetPosName  = "P0003";
    cmd.coord          = 2;  
    cmd.targetPosValue = std::vector<double>{240, 0.0, 380, 3.14, 0.0, 0.0};
    ret = job_insert_moveC(socket_fd, line, cmd);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 作业文件插入MoveC失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 作业文件插入MoveC成功" << std::endl;
    std::cout << std::endl;

    // ----获取所有作业文件名----
    std::cout << "[INFO] **********获取所有作业文件名**********" << std::endl;
    std::vector<std::vector<std::string>> AllJobNames;
    ret = job_get_all_jobfile_name(socket_fd, AllJobNames);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 获取作业文件名失败，程序退出!" << std::endl;
        return -1;
    }
    int i = 1;
    for (const auto& JobNames : AllJobNames) {
        for (const auto& name : JobNames) {
            std::cout << "[INFO] 作业文件 " << i++ << " 名称: " << name << std::endl;
        }
    }

    // ----切换为运行模式----
    // 运行作业文件需要在运行模式下执行（2=运行）
    std::cout << "--- 切换运行模式 ---" << std::endl;
    ret = set_current_mode(socket_fd, 2);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 切换运行模式失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 已切换为运行模式" << std::endl;

    // ----运行作业文件----
    std::cout << "[INFO] **********运行作业文件**********" << std::endl;
    ret = job_run(socket_fd, jobName);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 运行作业文件失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 运行作业文件成功" << std::endl;
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
