/**
 * test_error_callback_api.cpp
 * @brief 错误/警告回调示例程序
 * @attention
 *   - 确保控制器网络通信正常
 *   - 回调函数内不要做耗时操作或阻塞
 *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_error_callback
 */

#include "cpp/interface/tl_api.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

// ============================================================
// 回调函数（由用户实现，在收到错误/警告时被调用）
// ============================================================

/// @brief 错误/警告消息回调
/// @param messageType 消息类型（0=警告 1=错误）
/// @param message     消息内容
/// @param messageCode 消息码
void on_error_or_warning(int messageType, const char* message, int messageCode)
{
    const char* type_str = (messageType == 0) ? "警告" : "错误";
    std::cout << "\n>>> [回调触发] " << type_str
              << " | code=" << messageCode
              << " | " << message << std::endl;
}

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

    Result ret;

    // ============================================================
    // 1. 注册错误/警告回调
    // ============================================================
    print_separator("注册回调");
    ret = set_receive_error_or_warnning_message_callback(sock, on_error_or_warning);
    print_result("set_receive_error_or_warnning_message_callback", ret);
    if (ret != Result::SUCCESS) {
        std::cerr << "[ERROR] 注册回调失败" << std::endl;
        disconnect_robot(sock);
        return -1;
    }
    std::cout << "  [信息] 回调已注册，后续收到错误/警告时会自动触发" << std::endl;

    // ============================================================
    // 2. 逆解不可达点，触发错误回调
    // ============================================================
    // 通过 get_origin_coord_to_target_coord 尝试逆解一个超出工作空间的点，
    // 控制器会返回错误/警告，从而触发注册的回调函数
    print_separator("逆解不可达点");

    std::cout << "--- 逆解一个超出工作空间的点，观察回调触发 ---" << std::endl;
    {
        // 设置一个明显不可达的直角坐标点（X 方向超出机械臂最大臂展）
        std::vector<double> unreachable_pose = {99999, 0, 0, 0, 0, 0};
        std::vector<double> joint_result;
        bool convert_state = false;

        ret = get_origin_coord_to_target_coord(sock, 1, unreachable_pose, 0, joint_result, convert_state);
        print_result("逆解不可达点 (直角→关节)", ret);
        std::cout << "  输入直角坐标 [mm, rad]: X=99999, Y=0, Z=0" << std::endl;
        if (ret != Result::SUCCESS) {
            std::cout << "  [信息] 逆解失败，控制器的错误消息已通过回调输出" << std::endl;
        }
    }

    // 等待回调处理完成
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ============================================================
    // 3. 断开连接
    // ============================================================
    print_separator("断开连接");
    disconnect_robot(sock);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 错误回调示例程序运行完毕" << std::endl;
    return 0;
}
