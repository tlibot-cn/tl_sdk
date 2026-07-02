/**
 * test_coord_transform_api.cpp
 * @brief 坐标变换算法示例程序 —— 展示常用的坐标系转换接口
 * @attention
 *   - 需要控制器连接正常、已上电
 *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./test_coord_transform
 */

#include "cpp/interface/tl_api.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>

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

/// @brief 打印 double 向量
void print_vector(const std::string& name, const std::vector<double>& v, int width = 6)
{
    std::cout << "  " << name << " [" << v.size() << "] = {";
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << v[i];
        if (i < v.size() - 1) std::cout << ", ";
    }
    std::cout << "}" << std::endl;
}

/// @brief 打印矩阵（行主序）
void print_matrix(const std::string& name, const std::vector<double>& m, int rows, int cols)
{
    std::cout << "  " << name << " [" << m.size() << "]:" << std::endl;
    for (int r = 0; r < rows; ++r) {
        std::cout << "    ";
        for (int c = 0; c < cols; ++c) {
            int idx = r * cols + c;
            if (idx < (int)m.size()) {
                std::cout << std::fixed << std::setprecision(4) << std::setw(10) << m[idx] << " ";
            }
        }
        std::cout << std::endl;
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

    // ---- 双端口连接 ----
    print_separator("连接控制器");
    SOCKETFD sock = connect_robot("192.168.1.13", "6001");
    if (sock < 0) {
        std::cerr << "[ERROR] 连接 6001 端口失败" << std::endl;
        return -1;
    }
    SOCKETFD sock_servo = connect_robot("192.168.1.13", "7000");
    if (sock_servo < 0) {
        std::cerr << "[ERROR] 连接 7000 端口失败" << std::endl;
        disconnect_robot(sock);
        return -1;
    }
    std::cout << "[信息] 双端口连接成功 (6001=" << sock << ", 7000=" << sock_servo << ")" << std::endl;

    Result ret;

    // ---- 上电（servoJ 需要上电状态）----
    {
        print_separator("清错与上电");
        clear_error(sock);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        set_servo_poweroff(sock);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        int servo_state = -1;
        ret = get_servo_state(sock, servo_state);
        if (ret != Result::SUCCESS) {
            std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
            disconnect_robot(sock);
            disconnect_robot(sock_servo);
            return -1;
        }
        std::cout << "  [信息] 当前伺服状态: " << servo_state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

        if (servo_state == 2) {
            std::cerr << "[ERROR] 伺服处于报警状态，无法上电" << std::endl;
            disconnect_robot(sock);
            disconnect_robot(sock_servo);
            return -1;
        }

        if (servo_state != 3) {
            std::cout << "  [信息] 伺服未运行，执行上电流程..." << std::endl;
            ret = set_servo_state(sock, 1);
            if (ret != Result::SUCCESS && ret != Result::OPERATION_NOT_ALLOWED) {
                std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
                disconnect_robot(sock);
                disconnect_robot(sock_servo);
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            ret = set_servo_poweron(sock);
            if (ret != Result::SUCCESS) {
                std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
                disconnect_robot(sock);
                disconnect_robot(sock_servo);
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::cout << "  [信息] 上电成功" << std::endl;
        } else {
            std::cout << "  [信息] 伺服已运行 (servo_state=3)" << std::endl;
        }
    }

    // ============================================================
    // 1. 四元数 ↔ 欧拉角 (RPY) — 正反验证
    // ============================================================
    // 单位说明:
    //   四元数 [x, y, z, w]      — 无量纲（归一化）
    //   欧拉角 [roll, pitch, yaw] — 弧度 rad
    //   旋转矩阵 (3×3 行主序)     — 无量纲
    print_separator("四元数 ↔ 欧拉角");

    // 1a. 欧拉角 → 四元数 → 欧拉角（正反验证）
    {
        std::cout << "--- 1a. RPY ↔ 四元数 正反验证 ---" << std::endl;
        std::vector<double> rpy_in = {0.2, 0.3, 0.5}; // rad
        std::vector<double> quat, rpy_out;

        ret = get_rpy2quat(sock, rpy_in, quat);
        print_result("get_rpy2quat (RPY→四元数)", ret);
        if (ret != Result::SUCCESS) return -1;
        print_vector("  输入欧拉角 [rad]", rpy_in);
        print_vector("  中间四元数 [x, y, z, w]", quat);

        ret = get_quat2rpy(sock, quat, rpy_out);
        print_result("get_quat2rpy (四元数→RPY)", ret);
        if (ret != Result::SUCCESS) return -1;
        print_vector("  还原欧拉角 [rad]", rpy_out);

        if (rpy_out.size() >= 3 && rpy_in.size() >= 3) {
            double err = fabs(rpy_out[0] - rpy_in[0])
                       + fabs(rpy_out[1] - rpy_in[1])
                       + fabs(rpy_out[2] - rpy_in[2]);
            std::cout << "  正反变换误差 (应接近0): " << err << std::endl;
        }
    }

    // 1b. 欧拉角 → 旋转矩阵
    {
        std::cout << "\n--- 1b. get_rpy2r: 欧拉角转旋转矩阵 ---" << std::endl;
        std::vector<double> rpy = {0.0, 0.0, M_PI_2}; // rad
        std::vector<double> rot_mat;

        ret = get_rpy2r(sock, rpy, rot_mat);
        print_result("get_rpy2r", ret);
        print_vector("输入欧拉角 [rad]", rpy);
        if (ret == Result::SUCCESS) {
            print_matrix("输出旋转矩阵 (3x3, 行主序, 无量纲)", rot_mat, 3, 3);
        }
    }

    // ============================================================
    // 2. 位姿矩阵 ↔ 旋转矩阵 — 正反验证
    // ============================================================
    // 单位说明:
    //   旋转矩阵 R (3×3)     — 无量纲
    //   平移向量 T [Tx,Ty,Tz] — 毫米 mm
    //   位姿矩阵 TR (4×4)  — R 部分无量纲，T 部分单位 mm
    // 位姿矩阵 (行主序 16 元素):
    //   [Rxx Rxy Rxz Tx]  第 0 行
    //   [Ryx Ryy Ryz Ty]  第 1 行
    //   [Rzx Rzy Rzz Tz]  第 2 行
    //   [  0   0   0  1]  第 3 行
    print_separator("位姿矩阵 ↔ 旋转矩阵");

    // 2a. TR → R → TR（正反验证）
    {
        std::cout << "--- 2a. TR ↔ R 正反验证 ---" << std::endl;
        // 构造一个绕 Z 转 90°、平移 (100, 200, 300) mm 的位姿矩阵
        double cos90 = cos(M_PI_2), sin90 = sin(M_PI_2);
        std::vector<double> tr_in = {
            cos90, -sin90, 0.0, 100.0,
            sin90,  cos90, 0.0, 200.0,
            0.0,    0.0,   1.0, 300.0,
            0.0,    0.0,   0.0, 1.0
        };
        std::vector<double> R, tr_out;

        ret = get_tr2r(sock, tr_in, R);
        print_result("get_tr2r (TR→R)", ret);
        if (ret != Result::SUCCESS) return -1;
        print_matrix("输入位姿矩阵 (4x4, R无量纲, T:mm)", tr_in, 4, 4);
        print_matrix("中间旋转矩阵 (3x3, 无量纲)", R, 3, 3);

        ret = get_r2tr(sock, R, tr_out);
        print_result("get_r2tr (R→TR)", ret);
        if (ret != Result::SUCCESS) return -1;
        print_matrix("还原位姿矩阵 (4x4, R无量纲, T:mm)", tr_out, 4, 4);

        // 对比平移部分
        if (tr_out.size() >= 16 && tr_in.size() >= 16) {
            double t_err = fabs(tr_out[3] - tr_in[3])
                         + fabs(tr_out[7] - tr_in[7])
                         + fabs(tr_out[11] - tr_in[11]);
            std::cout << "  平移向量误差 [mm] (应接近0): " << t_err << std::endl;
        }
    }

    // ============================================================
    // 3. 坐标系转换（获取机器人当前位姿在不同坐标系下的表示）
    // ============================================================
    // 单位说明:
    //   关节坐标 [J1..J6] — 度 °
    //   直角坐标 [X,Y,Z,Rx,Ry,Rz] — 位移 mm，姿态 rad
    //   工具/用户坐标 — 同直角坐标
    print_separator("坐标系转换");
    {
        std::cout << "--- 3. get_origin_coord_to_target_coord ---" << std::endl;
        std::cout << "  坐标系: 0=关节(°)  1=直角(mm+rad)  2=工具(mm+rad)  3=用户(mm+rad)" << std::endl;

        // 从关节坐标系(0) 转换到直角坐标系(1)
        // 关节角度: J1=10°, J2=20°, J3=30°, J4=0°, J5=0°, J6=0°
        std::vector<double> joint_pos = {10, 20, 30, 0, 0, 0};
        std::vector<double> cartesian_pos;
        bool convert_state = false;

        ret = get_origin_coord_to_target_coord(sock, 0, joint_pos, 1, cartesian_pos, convert_state);
        print_result("get_origin_coord_to_target_coord (关节→直角) 正解", ret);
        print_vector("输入关节角度 [°]", joint_pos);
        if (ret == Result::SUCCESS) {
            std::cout << "  输出直角坐标 — 前3位为位移[mm]，后3位为姿态[rad]:" << std::endl;
            print_vector("  直角坐标", cartesian_pos);
            if (cartesian_pos.size() >= 6) {
                std::cout << "    位移 X=" << cartesian_pos[0] << "mm, Y=" << cartesian_pos[1]
                          << "mm, Z=" << cartesian_pos[2] << "mm" << std::endl;
                std::cout << "    姿态 Rx=" << cartesian_pos[3] << "rad, Ry=" << cartesian_pos[4]
                          << "rad, Rz=" << cartesian_pos[5] << "rad" << std::endl;
                std::cout << "    姿态换算(°): Rx=" << cartesian_pos[3] * 180.0 / M_PI
                          << ", Ry=" << cartesian_pos[4] * 180.0 / M_PI
                          << ", Rz=" << cartesian_pos[5] * 180.0 / M_PI << std::endl;
            }
        }

        // 逆解: 直角坐标系(1) → 关节坐标系(0)
        // 使用正解输出的直角坐标作为输入，验证正反解一致性
        if (ret == Result::SUCCESS && cartesian_pos.size() >= 6) {
            std::vector<double> joint_back;
            ret = get_origin_coord_to_target_coord(sock, 1, cartesian_pos, 0, joint_back, convert_state);
            print_result("get_origin_coord_to_target_coord (直角→关节) 逆解", ret);
            std::cout << "  输入直角坐标:" << std::endl;
            std::cout << "    位移 [mm]: X=" << cartesian_pos[0] << ", Y=" << cartesian_pos[1]
                      << ", Z=" << cartesian_pos[2] << std::endl;
            std::cout << "    姿态 [rad]: Rx=" << cartesian_pos[3] << ", Ry=" << cartesian_pos[4]
                      << ", Rz=" << cartesian_pos[5] << std::endl;
            if (ret == Result::SUCCESS) {
                print_vector("  逆解关节角度 [°]", joint_back);
                // 对比原始关节角度，计算误差
                double err = 0.0;
                for (size_t i = 0; i < joint_pos.size() && i < joint_back.size(); ++i) {
                    err += fabs(joint_pos[i] - joint_back[i]);
                }
                std::cout << "  正反解误差 (应接近0): " << err << std::endl;
            }
        }
    }

    // ============================================================
    // 4. 逆解关节坐标 → servoJ 下发运动
    // ============================================================
    // 演示流程: 给定直角坐标 → 逆解到关节角 → 通过 servoJ 发送到控制器
    print_separator("逆解 + servoJ 运动");

    // 取一个直角坐标点，逆解为关节角
    std::vector<double> target_pose = {300, 0, 300, 3.141, 0, 0}; // X=300mm, Y=0, Z=300mm
    std::vector<double> target_joints;
    bool convert_state2 = false;

    ret = get_origin_coord_to_target_coord(sock, 1, target_pose, 0, target_joints, convert_state2);
    print_result("逆解: 直角→关节", ret);
    if (ret == Result::SUCCESS) {
        print_vector("目标直角坐标 [mm, rad]", target_pose);
        print_vector("逆解关节角度 [°]", target_joints);

        // 通过 servoJ 发送关节角度到控制器
        // servoJ 需要连接 7000 端口，已在开头连接
        std::cout << "\n  --- 通过 servoJ 下发逆解结果 ---" << std::endl;

        // servoJ 约束参数（7 元素: 6 本体轴 + 1 外部轴）
        // 注意: SDK 底层协议要求 servoJ 相关向量固定为 7 元素
        std::vector<double> vmax = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0};
        std::vector<double> amax = {60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0};
        std::vector<double> jmax = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

        ret = open_servoJ(sock_servo, vmax, amax, jmax);
        print_result("open_servoJ", ret);
        if (ret == Result::SUCCESS) {
            // 逐步发送逆解出的关节角度，确保 servoJ 跟踪到位
            // 分 50 步从当前角度平滑过渡到目标角度
            // q 必须为 7 元素，与 open_servoJ 一致
            std::vector<double> current = {0, 0, 0, 0, 0, 0, 0};
            int steps = 50;
            for (int i = 0; i < steps; ++i) {
                std::vector<double> q(7);
                for (int j = 0; j < 6; ++j) {
                    q[j] = current[j] + (target_joints[j] - current[j]) * (i + 1) / steps;
                }
                q[6] = 0.0;  // 外部轴置零
                set_servoJ_pos(sock_servo, q);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // servoJ 是实时跟踪模式，等待固定时间确保运动到位
            std::cout << "  [信息] 等待 servoJ 运动到位..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));

            close_servoJ(sock_servo);
            print_result("close_servoJ", ret);
        }
    }

    // ---- 断开连接 ----
    print_separator("断开连接");
    disconnect_robot(sock);
    disconnect_robot(sock_servo);
    std::cout << "[信息] 已断开连接，机械臂保持运行模式" << std::endl;

    std::cout << "\n[信息] 坐标变换示例程序运行完毕" << std::endl;
    return 0;
}
