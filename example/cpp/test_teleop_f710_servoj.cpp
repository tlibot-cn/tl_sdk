/**
 * test_teleop_f710_servoj.cpp
 * @brief F710 手柄遥操作控制 (SDL2) — 直接 set_servoJ_pos 方案
 *
 * 适用场景:
 *   - 使用罗技 F710 手柄进行连续、平滑的实时遥操作
 *   - 在循环内直接 IK + set_servoJ_pos，每 4ms 稳定输出一帧 (250Hz)
 *   - 笛卡尔速度积分模式，手柄松手即停
 *
 * @attention
 *   - 确保机械臂供电正常、网络通信正常
 *   - 【重要】运行前请先在下方"参数设定区"确认并修改 ROBOT_IP（控制器IP）和 NUM_JOINTS（轴数）
 *   - 需要双端口连接: 6001（控制）+ 7000（伺服高频透传）
 *   - 手柄需切到 DirectInput 模式，开关拨到 "D"
 *   - 运行前请先安装 SDL2: sudo apt install libsdl2-dev
 *   - 示例会连接真实机械臂，请确保工作区域安全
 *   - 退出方式: Ctrl+C 或手柄 Back+Start
 *
 * @note 运行步骤
 *       编译: cd example/cpp/build && cmake .. && make
 *       链接动态库: export LD_LIBRARY_PATH=lib/x86:$LD_LIBRARY_PATH
 *       运行: ./test_teleop_f710_servoj
 *
 * @note 控制方式（笛卡尔速度映射模式）
 *       左摇杆 X/Y   → x / y 方向平移
 *       右摇杆 Y     → z 方向平移
 *       右摇杆 X     → rz 旋转（偏航，默认）
 *       LB + 右摇杆 X → rx 旋转（滚转）
 *       RB + 右摇杆 X → ry 旋转（俯仰）
 *       十字键上/下   → 速度倍率 ±5%
 *       A 键         → 回到零位
 *       Back+Start   → 紧急停止
 *
 * @note 启动流程
 *       双端口连接 → 伺服状态检查 → 上电 → 切换运行模式 → open_servoJ
 *       → 锁定当前关节角 → 进入遥操作主循环 (250Hz)
 *       退出: close_servoJ → 下电 → 断开双端口
 */

// ============================================================
// 头文件
// ============================================================

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <csignal>
#include <SDL2/SDL.h>
#include "cpp/interface/tl_api.h"

// ============================================================
// 【参数设定区】—— 按需修改
// ============================================================

// ---------- 机械臂连接参数 ----------
/// 控制器 IP 地址
static const std::string ROBOT_IP = "192.168.1.13";

/// ========== 轴数 ==========
/// 6 轴机械臂设为 6，7 轴设为 7
static const int NUM_JOINTS = 6;

/// ========== 零位关节角 ==========
/// [J1..JN]（度），N=NUM_JOINTS，程序启动时 FK 转为笛卡尔位姿
static const std::vector<double> HOME_JOINTS(NUM_JOINTS, 0.0);

// ---------- 控制周期 ----------
/// 控制周期（毫秒）——决定指令发送频率。ServoJ 最大支持 250Hz
static const int LOOP_MS = 4;   // 250Hz

/// ========== 运动速度（0-100） ==========
static const double SPEED_DEFAULT = 50.0;
static const double SPEED_MIN     =  5.0;
static const double SPEED_MAX     = 100.0;
static const double SPEED_STEP    = 5.0;

/// ========== 笛卡尔运动灵敏度（速度倍率 100% 时满推摇杆对应的速度）==========
/// 参考值: 真机 80 mm/s / 1.0 rad/s，仿真可适当提高
static const double POS_SENSITIVITY = 160.0;    // 平移 mm/s
static const double ROT_SENSITIVITY =  1.0;    // 旋转 rad/s

/// ========== 笛卡尔空间软限位 ==========
/// [x_min, x_max, y_min, y_max, z_min, z_max] (mm)
/// 超出时自动切断对应方向速度
static const std::vector<double> WORKSPACE_LIMITS = {
    -500.0, 500.0,    // x
    -500.0, 500.0,    // y
     0.0,   800.0,    // z（不碰到地面）
};

/// ========== 摇杆死区 ==========
static const double DEADZONE = 0.15;

/// ========== ServoJ 初始化参数（open_servoJ 传入控制器）==========
/// vmax 最大关节速度 (°/s)，最大 180；amax 加速度 (°/s²)；jmax 加加速度 (°/s³)
/// 参考值来自 ROS2 真机配置 tl_teleop_f710_6axis.yaml
static const double SERVO_VMAX = 300.0;       // °/s
static const double SERVO_AMAX = 3000.0;     // °/s²
static const double SERVO_JMAX = 50000.0;    // °/s³

/// ========== F710 轴映射（DirectInput 模式，开关拨到 "D"）==========
enum F710Axes {
    AXIS_LEFT_X  = 1,   // 左摇杆 X
    AXIS_LEFT_Y  = 0,   // 左摇杆 Y
    AXIS_RIGHT_X = 2,   // 右摇杆 X
    AXIS_RIGHT_Y = 3,   // 右摇杆 Y
    // 十字键用 SDL_JoystickGetHat(joystick, 0) 读取，非摇杆轴
};

/// ========== F710 按键映射 (DirectInput 模式) ==========
enum F710Buttons {
    BTN_X     = 0,
    BTN_A     = 1,
    BTN_B     = 2,
    BTN_Y     = 3,
    BTN_LB    = 4,
    BTN_RB    = 5,
    BTN_LT    = 6,
    BTN_RT    = 7,
    BTN_BACK  = 8,
    BTN_START = 9,
    NUM_BTNS  = 10,
};

/// ========== 摇杆 → 笛卡尔速度映射（默认模式，无 modifier 按键时）==========
/// 每个元素: {轴号, cart_index(0=x,1=y,2=z,3=rx,4=ry,5=rz), 方向(±1)}
/// LB 作为 modifier 时：右 X → rx（覆盖默认的 rz）
/// RB 作为 modifier 时：右 X → ry（覆盖默认的 rz）
struct AxisMap {
    int axis;
    int cart_idx;
    int sign;
};
static const AxisMap AXIS_MAP[] = {
    {AXIS_LEFT_X,  0,  -1},    // 左 X → x 平移
    {AXIS_LEFT_Y,  1,  -1},    // 左 Y → y 平移
    {AXIS_RIGHT_X, 5,  1},     // 右 X → rz 旋转（默认，LB 按住时改为 rx）
    {AXIS_RIGHT_Y, 2,  -1},    // 右 Y → z 平移  （默认，RB 按住时改为 ry）
};
static const int NUM_AXIS_MAPS = sizeof(AXIS_MAP) / sizeof(AXIS_MAP[0]);

// ============================================================
// 全局状态
// ============================================================

static std::atomic<bool> g_running{true};
static SOCKETFD g_sock_tcp = -1;
static SOCKETFD g_sock_udp = -1;

// ============================================================
// 信号处理
// ============================================================

static void signal_handler(int) {
    g_running.store(false);
}

// ============================================================
// 辅助函数
// ============================================================

/// SDL 轴值归一化，含死区
static double normalize_axis(Sint16 value, double deadzone) {
    double v = static_cast<double>(value) / 32767.0;
    if (v < -1.0) v = -1.0;
    if (std::abs(v) < deadzone) return 0.0;
    return (v > 0) ? (v - deadzone) / (1.0 - deadzone)
                   : (v + deadzone) / (1.0 - deadzone);
}

/// 工作空间限位保护
static void apply_workspace_limits(std::vector<double> &pose,
                                    const std::vector<double> &limits) {
    for (int i = 0; i < 3; ++i) {
        if (pose[i] < limits[i * 2])     pose[i] = limits[i * 2];
        if (pose[i] > limits[i * 2 + 1]) pose[i] = limits[i * 2 + 1];
    }
}

/// 打印当前笛卡尔位姿（单行覆盖刷新）
static void print_pose_state(const std::vector<double> &pose, double speed_pct) {
    std::cout << "\r  [速度 " << std::setw(3) << static_cast<int>(speed_pct)
              << "%]  x=" << std::fixed << std::setprecision(1) << pose[0]
              << "  y=" << pose[1] << "  z=" << pose[2]
              << "  rx=" << std::setprecision(2) << pose[3]
              << "  ry=" << pose[4] << "  rz=" << pose[5]
              << "  ";
    std::cout.flush();
}

// ============================================================
// 主函数
// ============================================================

int main(int argc, char *argv[]) {
    // ---- 信号 ----
    signal(SIGINT, signal_handler);

    std::cout << "=== F710 手柄遥操作（直接 ServoJ，250Hz 稳定输出） ===\n\n";

    // ---- 打印参数 ----
    std::cout << "控制周期: " << LOOP_MS << " ms (" << (1000/LOOP_MS) << " Hz)\n";
    std::cout << "平移灵敏度: " << POS_SENSITIVITY << " mm/s\n";
    std::cout << "旋转灵敏度: " << ROT_SENSITIVITY << " rad/s\n";
    std::cout << "死区:   " << DEADZONE << "\n";
    std::cout << "\n手柄操作说明:\n";
    std::cout << "  左摇杆 X/Y   →  x / y 平移\n";
    std::cout << "  右摇杆 Y     →  z 平移\n";
    std::cout << "  右摇杆 X     →  rz 旋转\n";
    std::cout << "  LB + 右摇杆 X →  rx 旋转\n";
    std::cout << "  RB + 右摇杆 X →  ry 旋转\n";
    std::cout << "  十字键上/下   →  速度倍率\n";
    std::cout << "  A 键         →  回到零位\n";
    std::cout << "  Back+Start   →  紧急停止\n\n";

    // ============================================================
    // 初始化 SDL
    // ============================================================

    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "SDL 初始化失败: " << SDL_GetError() << "\n";
        std::cerr << "请安装 SDL2: sudo apt install libsdl2-dev\n";
        return -1;
    }

    SDL_Joystick *joystick = SDL_JoystickOpen(0);
    if (!joystick) {
        std::cerr << "未检测到手柄，请连接 F710 后重试\n";
        SDL_Quit();
        return -1;
    }
    std::cout << "  已连接手柄: " << SDL_JoystickName(joystick) << "\n";
    std::cout << "  轴数: " << SDL_JoystickNumAxes(joystick)
              << ", 按键数: " << SDL_JoystickNumButtons(joystick) << "\n\n";

    // ============================================================
    // 连接机械臂
    // ============================================================

    g_sock_tcp = connect_robot(ROBOT_IP, "6001");
    g_sock_udp = connect_robot(ROBOT_IP, "7000");
    if (g_sock_tcp < 0 || g_sock_udp < 0) {
        std::cerr << "连接机械臂失败\n";
        SDL_JoystickClose(joystick);
        SDL_Quit();
        return -1;
    }
    std::cout << "  已连接到机械臂\n";

    // ---- 伺服状态检查与上电 ----
    std::cout << "\n--- 伺服状态检查与上电 ---\n";
    clear_error(g_sock_tcp);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    set_servo_poweroff(g_sock_tcp);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    {
        int servo_state = -1;
        auto ret_local = get_servo_state(g_sock_tcp, servo_state);
        if (ret_local != Result::SUCCESS) {
            std::cerr << "[ERROR] 获取伺服状态失败: " << ret_local << std::endl;
        }
        std::cout << "  当前伺服状态: " << servo_state
                  << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

        if (servo_state == 2) {
            std::cerr << "[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试" << std::endl;
        } else if (servo_state == 3) {
            std::cout << "  [信息] 伺服已运行，跳过上电" << std::endl;
        } else {
            // 确保伺服就绪
            if (servo_state == 0) {
                ret_local = set_servo_state(g_sock_tcp, 1);
                if (ret_local != Result::SUCCESS && ret_local != Result::OPERATION_NOT_ALLOWED) {
                    std::cerr << "[ERROR] set_servo_state(1) 失败，错误码: " << ret_local << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            // 上电使能
            ret_local = set_servo_poweron(g_sock_tcp);
            if (ret_local != Result::SUCCESS) {
                std::cerr << "[ERROR] set_servo_poweron 失败，错误码: " << ret_local << std::endl;
            } else {
                std::cout << "  [信息] 上电成功" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // ---- 切换运行模式 & 设置速度 ----
    set_current_mode(g_sock_tcp, 2);
    set_speed(g_sock_tcp, 30);

    // ---- 开启 ServoJ ----
    std::cout << "\n--- 开启伺服跟踪模式 ---\n";
    // 速度/加速度/加加速度约束（NUM_JOINTS 轴，单位 °/s, °/s², °/s³）
    std::vector<double> vmax_servo(NUM_JOINTS, SERVO_VMAX);
    std::vector<double> amax_servo(NUM_JOINTS, SERVO_AMAX);
    std::vector<double> jmax_servo(NUM_JOINTS, SERVO_JMAX);
    Result ret = open_servoJ(g_sock_udp, vmax_servo, amax_servo, jmax_servo);
    if (ret != Result::SUCCESS) {
        std::cerr << "open_servoJ 失败\n";
        disconnect_robot(g_sock_udp);
        disconnect_robot(g_sock_tcp);
        SDL_JoystickClose(joystick);
        SDL_Quit();
        return -1;
    }

    // ---- 获取当前关节角，立即发送稳住（避免 servoJ 空窗期）----
    std::vector<double> joint_cmd(NUM_JOINTS, 0.0);
    get_current_position(g_sock_tcp, 0, joint_cmd);
    set_servoJ_pos(g_sock_udp, joint_cmd);
    std::cout << "  初始关节角已锁定，等待手柄输入" << std::endl;

    // ---- 用实际关节角作为 IK 参考点（避免全零导致解跳变）----
    std::vector<double> ref_pos = joint_cmd;

    // ---- FK 零位关节角 → 笛卡尔位姿（归零目标）----
    std::vector<double> home_cart(7, 0.0);
    {
        bool convert_state = false;
        ret = get_origin_coord_to_target_coord(
            g_sock_tcp, 0, HOME_JOINTS,
            1, home_cart, convert_state, 0, ref_pos);
    }
    if (ret != Result::SUCCESS || home_cart.size() < 6) {
        std::cerr << "FK 零位关节角失败\n";
        for (int i = 0; i < 6; ++i) home_cart[i] = 0.0;
    }

    // ---- 获取当前笛卡尔位姿作为初始目标 ---- 
    std::vector<double> target_pose(6, 0.0);
    std::vector<double> current_cart(7, 0.0);
    ret = get_current_position(g_sock_tcp, 1, current_cart);
    if (ret == Result::SUCCESS && current_cart.size() >= 6) {
        for (int i = 0; i < 6; ++i) target_pose[i] = current_cart[i];
    } else {
        for (int i = 0; i < 6; ++i) target_pose[i] = home_cart[i];
    }

    // ============================================================
    // 主控制循环（250Hz 稳定输出，直接 set_servoJ_pos）
    // ============================================================
    //
    // 与 06 方案的关键区别：
    //   1. 不使用 servo_movel（阻塞、有空档期）
    //   2. 自行 IK + set_servoJ_pos，每 10ms 稳定一帧
    //   3. dt 基于实际 LOOP_MS，速度计算准确
    //   4. sleep_until 累加定时，超时则重置防止累积漂移
    //
    // ============================================================

    std::cout << "\n=== 进入遥操作模式 (Ctrl+C 或 Back+Start 退出) ===\n\n";

    double speed_pct = SPEED_DEFAULT;
    bool prev_dpad_up = false, prev_dpad_down = false;
    bool prev_a_btn = false;
    bool force_ik = false;  // 用于 A 键归零时强制做一次 IK
    bool btn_state[NUM_BTNS] = {false};

    const auto period = std::chrono::milliseconds(LOOP_MS);
    auto next_time = std::chrono::steady_clock::now();
    auto last_time = next_time;

    while (g_running.load()) {
        // ============================================================
        // 1. 读取手柄
        // ============================================================

        SDL_JoystickUpdate();

        Sint16 axis_raw[4];
        axis_raw[AXIS_LEFT_X]  = SDL_JoystickGetAxis(joystick, AXIS_LEFT_X);
        axis_raw[AXIS_LEFT_Y]  = SDL_JoystickGetAxis(joystick, AXIS_LEFT_Y);
        axis_raw[AXIS_RIGHT_X] = SDL_JoystickGetAxis(joystick, AXIS_RIGHT_X);
        axis_raw[AXIS_RIGHT_Y] = SDL_JoystickGetAxis(joystick, AXIS_RIGHT_Y);

        Uint8 hat = SDL_JoystickGetHat(joystick, 0);
        bool dpad_up   = (hat & SDL_HAT_UP)   != 0;
        bool dpad_down = (hat & SDL_HAT_DOWN) != 0;

        for (int b = 0; b < NUM_BTNS; ++b)
            btn_state[b] = SDL_JoystickGetButton(joystick, b);

        // ============================================================
        // 2. 紧急停止
        // ============================================================
        if (btn_state[BTN_BACK] && btn_state[BTN_START]) {
            std::cout << "\n  紧急停止！\n";
            break;
        }

        // ============================================================
        // 3. 速度倍率（十字键 Hat 边沿触发）
        // ============================================================
        if (dpad_up && !prev_dpad_up)
            speed_pct = std::min(speed_pct + SPEED_STEP, SPEED_MAX);
        if (dpad_down && !prev_dpad_down)
            speed_pct = std::max(speed_pct - SPEED_STEP, SPEED_MIN);
        prev_dpad_up = dpad_up;
        prev_dpad_down = dpad_down;

        // ============================================================
        // 4. 回到零位（A 键边沿触发）
        // ============================================================
        // 不自行插值，而是将目标位姿设为 home_cart 并强制做一次 IK，
        // 随后主循环按 250Hz 持续发送 HOME_JOINTS，
        // 实际速度由 open_servoJ 的 SERVO_VMAX 限制，平滑无回弹。
        if (btn_state[BTN_A] && !prev_a_btn) {
            std::cout << "\n  回到零位...\n";
            speed_pct = 50.0;
            for (int i = 0; i < 6; ++i) target_pose[i] = home_cart[i];
            force_ik = true;
        }
        prev_a_btn = btn_state[BTN_A];

        // ============================================================
        // 5. 计算笛卡尔速度 → 累加目标位姿
        // ============================================================
        //
        // dt 使用实际帧间隔（而非固定 LOOP_MS），确保即使 IK 调用耗时较长，
        // 每帧积分量仍与真实经过时间匹配，速度感不受循环频率影响。
        //
        double speed_scale = speed_pct / 100.0;
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;
        // 首帧或长时间暂停后 dt 可能偏大，限幅避免突变
        if (dt > 0.1) dt = LOOP_MS / 1000.0;

        // 5a. 摇杆默认映射
        double cart_vel[6] = {0.0};
        for (int i = 0; i < NUM_AXIS_MAPS; ++i) {
            double raw = normalize_axis(axis_raw[AXIS_MAP[i].axis], DEADZONE);
            cart_vel[AXIS_MAP[i].cart_idx] += raw * AXIS_MAP[i].sign;
        }
        // 5b. Modifier 按键覆盖
        double rx_raw = normalize_axis(axis_raw[AXIS_RIGHT_X], DEADZONE);
        double ry_raw = normalize_axis(axis_raw[AXIS_RIGHT_Y], DEADZONE);
        if (btn_state[BTN_LB]) {
            cart_vel[3] += rx_raw * 1.0;   // rx
            cart_vel[5] -= rx_raw * 1.0;   // 取消默认 rz
        }
        if (btn_state[BTN_RB]) {
            // RB + 右 X → ry（取消默认的 rz）
            cart_vel[4] += rx_raw * 1.0;   // ry
            cart_vel[5] -= rx_raw * 1.0;   // 取消默认 rz
        }
        // 5c. 积分
        for (int j = 0; j < 3; ++j)
            target_pose[j] += cart_vel[j] * speed_scale * POS_SENSITIVITY * dt;
        for (int j = 3; j < 6; ++j)
            target_pose[j] += cart_vel[j] * speed_scale * ROT_SENSITIVITY * dt;

        // ============================================================
        // 6. 工作空间限位保护
        // ============================================================
        apply_workspace_limits(target_pose, WORKSPACE_LIMITS);

        // ============================================================
        // 7. 发送 & IK 更新
        // ============================================================
        //
        // 【关键设计】先发送、后 IK：
        //   1) set_servoJ_pos (UDP，~1ms) 立即发出上一帧的关节角，
        //      保证机械臂始终有指令流，不会因 IK 等待而断档。
        //   2) 随后 get_origin_coord_to_target_coord (TCP，可能较慢)
        //      计算新关节角供下一帧使用。
        //   3) 无摇杆输入时跳过 IK（target_pose 未变，IK 结果相同），
        //      让循环维持在 250Hz，松手后立即停。
        //
        // 7a. 立即发送上一帧的关节角（保持指令流不断）
        set_servoJ_pos(g_sock_udp, joint_cmd);

        // 7b. 有摇杆输入或强制 IK（A 键归零）时才做 IK
        bool has_input = false;
        for (int i = 0; i < 6; ++i) {
            if (std::abs(cart_vel[i]) > 1e-6) { has_input = true; break; }
        }

        if (has_input || force_ik) {
            force_ik = false;
            std::vector<double> new_joint(NUM_JOINTS, 0.0);
            bool convert_state = false;
            ret = get_origin_coord_to_target_coord(
                g_sock_tcp, 1, target_pose,
                0, new_joint, convert_state, 0, ref_pos);
            if (ret == Result::SUCCESS) {
                joint_cmd.swap(new_joint);
                ref_pos = joint_cmd;
            }
            // IK 失败则 joint_cmd 保持原值（上一帧的有效值）
        }

        // ============================================================
        // 8. 打印状态
        // ============================================================
        print_pose_state(target_pose, speed_pct);

        // ============================================================
        // 9. 累加定时，250Hz 稳定输出
        // ============================================================
        next_time += period;
        // 如果处理耗时超过了周期，next_time 已落后于实际时间，
        // 此时直接进入下一帧而不等待，并将 next_time 重置到当前时间，
        // 避免累积漂移导致循环"空转"无等待
        if (next_time < std::chrono::steady_clock::now()) {
            next_time = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_until(next_time);
    }

    // ============================================================
    // 清理退出
    // ============================================================

    std::cout << "\n\n--- 退出中... ---\n";

    close_servoJ(g_sock_udp);
    set_current_mode(g_sock_tcp, 0);
    set_servo_poweroff(g_sock_tcp);

    disconnect_robot(g_sock_udp);
    disconnect_robot(g_sock_tcp);

    SDL_JoystickClose(joystick);
    SDL_Quit();

    std::cout << "=== F710 遥操作结束 ===\n";
    return 0;
}
