/**
 * test_queue_motion_api.cpp
 * @brief 队列运动示例程序 —— 展示队列运动完整工作流
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
 *   - 队列运动模式下，所有指令先在本地缓存，调用 send_to_controller 后一次性下发执行
 *   - 演示三种队列运动：纯 MoveJ、纯 MoveL、MoveJ+MoveL 混合
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_queue_motion
 */

#include "cpp/interface/tl_api.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <functional>

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

/**
 * @brief 构建 MoveCmd（关节坐标模式）
 * @param joints 6 个关节角度值，单位 °（度），顺序 [J1, J2, J3, J4, J5, J6]
 * @param vel    百分比 % (1,100]，最大关节速度的百分比，实际受全局速度限制
 * @param acc    百分比 % (1,100]
 * @param dec    百分比 % (1,100]
 * @param pl     平滑参数 [0,5]
 * @note 实际运动速度 = 全局速度 × (vel / 100)
 */
MoveCmd make_movej_cmd(const std::vector<double>& joints,
                       double vel = 30, double acc = 50, double dec = 50, int pl = 0)
{
    MoveCmd cmd;
    cmd.coord        = 0; // 关节坐标系
    cmd.velocity     = vel;
    cmd.acc          = acc;
    cmd.dec          = dec;
    cmd.pl           = pl;
    cmd.targetPosValue.resize(7, 0.0);
    for (size_t i = 0; i < joints.size() && i < 7; ++i) {
        cmd.targetPosValue[i] = joints[i];
    }
    return cmd;
}

/**
 * @brief 构建 MoveCmd（直角坐标模式）
 * @param pose 位姿 [X, Y, Z, Rx, Ry, Rz]，位置单位 mm，姿态单位 °
 * @param vel    百分比 % (1,1000]，最大 TCP 线速度的百分比
 * @param acc    百分比 % (1,100]
 * @param dec    百分比 % (1,100]
 * @param pl     平滑参数 [0,5]
 * @note 实际运动速度 = 全局速度 × (vel / 100)
 */
MoveCmd make_movel_cmd(const std::vector<double>& pose,
                       double vel = 100, double acc = 50, double dec = 50, int pl = 0)
{
    MoveCmd cmd;
    cmd.coord        = 1; // 直角坐标系，可切换到 2：工具坐标系，3：用户坐标系
    cmd.velocity     = vel;
    cmd.acc          = acc;
    cmd.dec          = dec;
    cmd.pl           = pl;
    cmd.targetPosValue.resize(7, 0.0);
    for (size_t i = 0; i < pose.size() && i < 7; ++i) {
        cmd.targetPosValue[i] = pose[i];
    }
    return cmd;
}

// ============================================================
// 队列运动执行器
// ============================================================

/**
 * @brief 执行一轮队列运动
 * @param sock       socket 句柄
 * @param name       队列名称（用于打印）
 * @param fn_push    回调函数，在此函数中调用 queue_motion_push_back_*
 *                   回调会在队列模式开启后、发送前被调用
 * @return true 成功 / false 失败
 */
bool run_queue_motion(SOCKETFD sock, const std::string& name,
                      std::function<bool(SOCKETFD)> fn_push)
{
    print_separator("队列运动：" + name);

    // 1. 打开队列运动模式（会清空远端队列）
    Result ret = queue_motion_set_status(sock, true);
    print_result("queue_motion_set_status(true)", ret);
    if (ret != Result::SUCCESS) return false;

    // 确认队列模式已开启
    {
        bool status = false;
        queue_motion_get_status(sock, status);
        std::cout << "  [信息] 队列模式状态: " << (status ? "已开启" : "已关闭") << std::endl;
    }

    // 2. 插入运动指令
    std::cout << "  [信息] 插入运动指令..." << std::endl;
    if (!fn_push(sock)) {
        std::cerr << "[ERROR] 插入运动指令失败" << std::endl;
        queue_motion_set_status(sock, false);
        return false;
    }

    // 3. 查询本地队列长度
    {
        int size = 0;
        ret = queue_motion_size(sock, size);
        if (ret == Result::SUCCESS) {
            std::cout << "  [信息] 本地队列长度: " << size << " 条指令" << std::endl;
        }
    }

    // 4. 发送队列到控制器并等待执行完成
    std::cout << "  [信息] 发送队列到控制器..." << std::endl;
    ret = queue_motion_send_to_controller(sock, 0, false);
    print_result("send_to_controller", ret);
    if (ret != Result::SUCCESS) {
        queue_motion_set_status(sock, false);
        return false;
    }

    std::cout << "  [信息] 等待运动执行完成..." << std::endl;
    while (true) {
        int running_state = -1;
        ret = get_robot_running_state(sock, running_state);
        if (ret == Result::SUCCESS && running_state == 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "  [信息] 队列运动执行完毕" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 5. 关闭队列运动模式
    queue_motion_set_status(sock, false);
    return true;
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

    // ---- 清错 ----
    clear_error(sock);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ---- 切换为运行模式 ----
    // 队列运动需要在运行模式下执行（2=运行），切换后控制器通常自动上电
    std::cout << "--- 切换运行模式 ---\n";
    Result ret = set_current_mode(sock, 2);
    if (ret == Result::SUCCESS) {
        std::cout << "  [信息] 已切换为运行模式\n";
    } else {
        print_result("set_current_mode(2)", ret);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ---- 检查伺服状态，未上电则手动上电 ----
    {
        int servo_state = -1;
        ret = get_servo_state(sock, servo_state);
        if (ret == Result::SUCCESS && servo_state == 3) {
            std::cout << "  [信息] 伺服已运行（servo_state=3），无需重复上电\n";
        } else {
            std::cout << "  [信息] 伺服状态=" << servo_state << "，手动上电\n";
            ret = set_servo_state(sock, 1);
            if (ret != Result::SUCCESS && ret != Result::OPERATION_NOT_ALLOWED) {
                std::cerr << "[ERROR] 设置伺服状态失败: " << ret << std::endl;
                disconnect_robot(sock);
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            ret = set_servo_poweron(sock);
            if (ret != Result::SUCCESS) {
                std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
                disconnect_robot(sock);
                return -1;
            }
            std::cout << "  [信息] 上电成功\n";
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ---- 设置全局速度 ----
    // 控制器默认全局速度为 5%，设为 30% 提升运动效率
    print_separator("设置全局速度");
    ret = set_speed(sock, 30);
    print_result("set_speed(30)", ret);

    // ============================================================
    // 演示 1: 纯 MoveJ 队列运动
    // ============================================================
    run_queue_motion(sock, "纯 MoveJ", [](SOCKETFD sock) -> bool {
        Result r;

        std::cout << "  push_back_moveJ:  J1=30°" << std::endl;
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd({30, 0, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != Result::SUCCESS) return false;

        std::cout << "  push_back_moveJ:  J1=30°, J2=20°" << std::endl;
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd({30, 20, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != Result::SUCCESS) return false;

        std::cout << "  push_back_moveJ:  J1=0°, J2=20°" << std::endl;
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd({0, 20, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != Result::SUCCESS) return false;

        std::cout << "  push_back_moveJ:  回零" << std::endl;
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd({0, 0, 0, 0, 0, 0}, 20));
        print_result("push_back_moveJ", r);
        if (r != Result::SUCCESS) return false;

        return true;
    });

    // ============================================================
    // 演示 2: 纯 MoveL 队列运动
    // ============================================================
    run_queue_motion(sock, "纯 MoveL", [](SOCKETFD sock) -> bool {
        Result r;

        std::cout << "  push_back_moveL:  X=300, Y=0, Z=300" << std::endl;
        r = queue_motion_push_back_moveL(sock, make_movel_cmd({300, 0, 300, 3.141, 0, 0}, 30));
        print_result("push_back_moveL", r);
        if (r != Result::SUCCESS) return false;

        std::cout << "  push_back_moveL:  X=400, Y=100, Z=300" << std::endl;
        r = queue_motion_push_back_moveL(sock, make_movel_cmd({400, 100, 300, 3.141, 0, 0}, 30));
        print_result("push_back_moveL", r);
        if (r != Result::SUCCESS) return false;

        std::cout << "  push_back_moveL:  X=300, Y=200, Z=300" << std::endl;
        r = queue_motion_push_back_moveL(sock, make_movel_cmd({300, 200, 300, 3.141, 0, 0}, 30));
        print_result("push_back_moveL", r);
        if (r != Result::SUCCESS) return false;

        std::cout << "  push_back_moveL:  回起始点" << std::endl;
        r = queue_motion_push_back_moveL(sock, make_movel_cmd({229.882, 0, 358.680, 3.141, 0, 0}, 20));
        print_result("push_back_moveL", r);
        if (r != Result::SUCCESS) return false;

        return true;
    });

    // ============================================================
    // 演示 3: MoveJ + MoveL 混合队列运动
    // ============================================================
    run_queue_motion(sock, "MoveJ + MoveL 混合", [](SOCKETFD sock) -> bool {
        Result r;

        // 先用 MoveJ 调整姿态
        std::cout << "  push_back_moveJ:  J1=45°（调整关节姿态）" << std::endl;
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd({45, 0, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != Result::SUCCESS) return false;

        // 再用 MoveL 走直线
        std::cout << "  push_back_moveL:  X=400, Y=0, Z=300" << std::endl;
        r = queue_motion_push_back_moveL(sock, make_movel_cmd({400, 0, 300, 3.141, 0, 0}, 30));
        print_result("push_back_moveL", r);
        if (r != Result::SUCCESS) return false;

        // MoveJ 换向
        std::cout << "  push_back_moveJ:  J1=-45°（换向）" << std::endl;
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd({-45, 0, 0, 0, 0, 0}, 30));
        print_result("push_back_moveJ", r);
        if (r != Result::SUCCESS) return false;

        // MoveL 走另一条直线
        std::cout << "  push_back_moveL:  X=300, Y=200, Z=300" << std::endl;
        r = queue_motion_push_back_moveL(sock, make_movel_cmd({300, 200, 300, 3.141, 0, 0}, 30));
        print_result("push_back_moveL", r);
        if (r != Result::SUCCESS) return false;

        // MoveJ 回零
        std::cout << "  push_back_moveJ:  回零" << std::endl;
        r = queue_motion_push_back_moveJ(sock, make_movej_cmd({0, 0, 0, 0, 0, 0}, 20));
        print_result("push_back_moveJ", r);
        if (r != Result::SUCCESS) return false;

        return true;
    });

    // ---- 断开连接（不下电，保持运行模式）----
    print_separator("断开连接");
    disconnect_robot(sock);
    std::cout << "[信息] 已断开连接，机械臂保持运行模式" << std::endl;

    std::cout << "\n[信息] 队列运动示例程序运行完毕" << std::endl;
    return 0;
}
