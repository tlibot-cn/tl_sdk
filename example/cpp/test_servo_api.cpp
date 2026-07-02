/**
 * test_servo_api.cpp
 * @brief 伺服状态与速度控制示例程序
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
 *   - 运行模式：示教模式
 *   - 伺服状态机: 0=停止 → 1=就绪 → 3=运行（上电后）
 *   - set_servo_state(1) 必须先于 set_servo_poweron 调用
 *   - set_servo_poweroff 只能在 state=3 时调用，成功后回到 state=1
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_servo
 */

#include "cpp/interface/tl_api.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <string>

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

/// @brief 获取伺服状态文本描述
const char* servo_state_text(int state)
{
    switch (state) {
        case 0:  return "0=停止";
        case 1:  return "1=就绪";
        case 2:  return "2=报警";
        case 3:  return "3=运行";
        default: return "未知";
    }
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

    // 声明 ret 变量供后续使用
    Result ret;

    // ============================================================
    // 1. 查询初始伺服状态
    // ============================================================
    print_separator("查询初始伺服状态");

    {
        int state = -1;
        ret = get_servo_state(sock, state);
        if (ret == Result::SUCCESS) {
            std::cout << "  [信息] 初始伺服状态: " << servo_state_text(state) << std::endl;
        } else {
            print_result("get_servo_state", ret);
        }
    }

    // ============================================================
    // 2. 切换为示教模式
    // ============================================================
    // 伺服状态操作通常在示教模式下进行（0=示教）
    print_separator("切换示教模式");
    ret = set_current_mode(sock, 0);
    print_result("set_current_mode(0)", ret);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ============================================================
    // 3. 清错处理（在设置伺服状态前先清除控制器错误和报警）
    // ============================================================
    print_separator("清错");
    ret = clear_error(sock);
    print_result("clear_error", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    ret = set_servo_poweroff(sock);
    print_result("set_servo_poweroff", ret);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 检查伺服状态
    {
        int state = -1;
        ret = get_servo_state(sock, state);
        if (ret != Result::SUCCESS) {
            std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
            disconnect_robot(sock);
            return -1;
        }
        std::cout << "  [信息] 当前伺服状态: " << servo_state_text(state) << std::endl;

        if (state == 2) {
            // state=2 为报警状态，此时无法上电
            std::cerr << "[ERROR] 伺服处于报警状态，无法继续上电，请排查报警原因后重试" << std::endl;
            disconnect_robot(sock);
            return -1;
        }
    }

    // ============================================================
    // 4. 设置伺服就绪 → 上电 → 验证状态变化
    // ============================================================
    print_separator("伺服上电流程");

    // 4a. 设置伺服就绪（state=1）
    std::cout << "[步骤 4a] 设置伺服就绪 (set_servo_state=1)" << std::endl;
    ret = set_servo_state(sock, 1);
    if (ret == Result::OPERATION_NOT_ALLOWED) {
        std::cout << "  [信息] 机械臂已处于就绪或运行状态，无需设置" << std::endl;
    } else {
        print_result("set_servo_state(1)", ret);
        if (ret != Result::SUCCESS) {
            disconnect_robot(sock);
            return -1;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 验证当前状态
    {
        int state = -1;
        get_servo_state(sock, state);
        std::cout << "  [信息] 当前伺服状态: " << servo_state_text(state) << std::endl;
    }

    // 4b. 上电使能（state=1 → state=3）
    std::cout << "\n[步骤 4b] 上电使能 (set_servo_poweron)" << std::endl;
    ret = set_servo_poweron(sock);
    print_result("set_servo_poweron", ret);
    if (ret != Result::SUCCESS) {
        disconnect_robot(sock);
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 验证上电成功后的状态应为 3=运行
    {
        int state = -1;
        ret = get_servo_state(sock, state);
        if (ret == Result::SUCCESS) {
            std::cout << "  [信息] 上电后伺服状态: " << servo_state_text(state);
            if (state == 3) {
                std::cout << "（上电成功）";
            }
            std::cout << std::endl;
        }
    }

    // ============================================================
    // 4. 速度查询与设置
    // ============================================================
    print_separator("速度查询与设置");

    // 4a. 查询当前速度
    {
        int speed = -1;
        ret = get_speed(sock, speed);
        if (ret == Result::SUCCESS) {
            std::cout << "  [信息] 当前全局速度: " << speed << "%" << std::endl;
        } else {
            print_result("get_speed", ret);
        }
    }

    // 4b. 设置速度并验证
    // set_speed 范围 0<speed≤100，设置后影响后续所有运动的速度上限
    int test_speeds[] = {50, 80, 30};
    for (int spd : test_speeds) {
        std::cout << "\n[步骤 4] 设置全局速度: " << spd << "%" << std::endl;
        ret = set_speed(sock, spd);
        print_result("set_speed", ret);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        int current_speed = -1;
        ret = get_speed(sock, current_speed);
        if (ret == Result::SUCCESS) {
            std::cout << "  [信息] 读取确认: 当前速度 = " << current_speed << "%" << std::endl;
        }
    }

    // ============================================================
    // 5. 下电 → 停止 → 验证状态变化
    // ============================================================
    print_separator("伺服下电流程");

    // 5a. 下电（state=3 → state=1）
    std::cout << "[步骤 5a] 下电 (set_servo_poweroff)" << std::endl;
    ret = set_servo_poweroff(sock);
    if (ret == Result::SUCCESS) {
        std::cout << "  [成功] set_servo_poweroff" << std::endl;
    } else if (ret == Result::RECEIVE_FAILED) {
        // 下电后网络可能自动断开，-1 视为正常
        std::cout << "  [信息] set_servo_poweroff 返回 -1（网络可能已断开）" << std::endl;
    } else {
        print_result("set_servo_poweroff", ret);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 5b. 验证下电后的伺服状态
    {
        int state = -1;
        ret = get_servo_state(sock, state);
        if (ret == Result::SUCCESS) {
            std::cout << "  [信息] 下电后伺服状态: " << servo_state_text(state) << std::endl;
        }
    }

    // 5c. 设置伺服停止（state=1 → state=0）
    std::cout << "\n[步骤 5c] 设置伺服停止 (set_servo_state=0)" << std::endl;
    ret = set_servo_state(sock, 0);
    if (ret == Result::OPERATION_NOT_ALLOWED) {
        std::cout << "  [信息] 当前状态不允许直接设为停止" << std::endl;
    } else {
        print_result("set_servo_state(0)", ret);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 最终状态确认
    {
        int state = -1;
        ret = get_servo_state(sock, state);
        if (ret == Result::SUCCESS) {
            std::cout << "  [信息] 最终伺服状态: " << servo_state_text(state) << std::endl;
        }
    }

    // ============================================================
    // 6. 断开连接
    // ============================================================
    print_separator("断开连接");
    disconnect_robot(sock);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 伺服状态示例程序运行完毕" << std::endl;
    return 0;
}
