# TL Robot Arm SDK — Linux/x86_64 C++

## 项目概述

机械臂控制 SDK，提供 C++ 和 C 双语言 API，通过 TCP/IP 远程控制机器人控制器。Python 绑定由 SWIG 自动生成。

**思考与输出语言**: 所有思考过程和输出必须使用中文。

## 构建与运行

### 编译示例

#### CMake（推荐）

```bash
cd example/cpp/build
cmake ..
make
```

- 自动扫描 `example/cpp/` 下所有 `.cpp` 源文件生成可执行文件
- 自动嵌入 RPATH，运行时无需设置 `LD_LIBRARY_PATH`
- 直接运行 `./test_connect_api`、`./test_servo_api` 等即可（可执行文件名对应源文件名，如 `test_servo_api.cpp` 生成 `test_servo_api`）

### 运行前设置

```bash
export LD_LIBRARY_PATH=lib/x86:$LD_LIBRARY_PATH
example/cpp/build/test_connect_api
```

## 关键架构

### 双端口连接模式（务必注意）

| 端口 | 用途 | 连接函数 |
|---|---|---|
| 6001 | 控制/状态（运动指令、IO、作业管理） | `connect_robot(ip, "6001")` |
| 7000 | 伺服高频模式（servoJ 实时跟踪） | `connect_robot(ip, "7000")` |

- **6001** → `SOCKETFD` 用于 `robot_movej`、`set_servo_state`、`job_create`、`modbus_*` 等绝大多数接口
- **7000** → 专用于 `open_servoJ`/`set_servoJ_pos`/`close_servoJ`（10ms 周期高频透传）
- 断开顺序: `set_servo_poweroff` → `disconnect_robot`（下电可能自动断连，返回 -1 也视为成功）

### 上电流程（必须两步）

```cpp
set_servo_state(sock, 1);          // 步骤1: 伺服就绪
sleep_for(milliseconds(500));
set_servo_poweron(sock);           // 步骤2: 上电使能
```

### 错误码（`Result` 枚举）

| 值 | 含义 |
|---|---|
| 0 (SUCCESS) | 成功 |
| -1 (RECEIVE_FAILED) | 接收失败 |
| -2 (DISCONNECT) | 未连接 |
| -3 (PARAM_ERR) | 参数错误 |
| -4 (OPERATION_NOT_ALLOWED) | 操作不允许（如重复上电/下电） |
| -5 (EXCEPTION) | 异常 |
| -6 (TIMEOUT) | 超时 |

下电时返回 -1 也视为成功（网络同步断开）。

## 编码风格参考

以下规范以 `example/cpp/test_connect_api.cpp`、`test_info_query_api.cpp`、`test_log_download_api.cpp` 三个文件为首要参考源。

### 文件头注释格式

每个 `.cpp` 文件顶部使用 Doxygen 风格注释块：

```cpp
/**
 * filename.cpp
 * @brief 一句话描述功能
 * @attention 运行前提条件/注意事项
 * @note 运行步骤
 *       编译: g++ -std=c++11 filename.cpp -o output -I./include ./lib/x86/_tl_host.so -lpthread
 *       链接动态库: export LD_LIBRARY_PATH=lib/x86:$LD_LIBRARY_PATH
 *       运行: ./output
 */
```

### 注释规范

- **语言**: 简体中文（全文统一使用中文注释）
- **文件头**: 使用 `/** ... */` Doxygen 块，含 `@brief`、`@attention`、`@note`
- **函数签名**: Doxygen 风格 `/** @brief ... @param ... @return ... @warning ... */`
- **行内注释**: `//` 解释每段代码的作用，中文描述
- **节标题**: `// ----功能描述----` 用 `----` 包裹
- **结构体字段**: 逐行行尾注释说明含义和取值范围
- **日志输出**: `[INFO]` 表示正常信息，`[ERROR]` 表示错误信息
- **复杂约束**使用 `@note` / `@attention` / `@warning` 标注

```cpp
// 文件头示例 (Doxygen)
/**
 * @brief 设置伺服状态
 * @param state 0 停止 / 1 就绪
 * @warning 需确保系统无错误后再调用
 */
EXPORT_API Result set_servo_state(SOCKETFD socketFd, int state);

// 行内注释示例
// 定义IP地址和端口
std::string ip = "192.168.1.13";

// 节标题示例
// ----获取机械臂当前坐标----
```

### 代码风格

| 项目 | 规范 | 示例 |
|---|---|---|
| 头文件包含顺序 | 标准库 → 项目头文件 | `#include <iostream>` → `#include "cpp/interface/tl_api.h"` |
| 函数命名 | `snake_case`（全局函数/SDK API 均一致） | `enable_servo`, `connect_robot`, `set_servo_state` |
| 变量命名 | `snake_case` 优先，偶用 `camelCase` | `socket_fd`（推荐），`axisNum`（偶见） |
| 返回类型 | `auto` 推导 + `Result` 枚举比较 | `auto ret = ...; if (ret != Result::SUCCESS)` |
| 错误处理 | 早期返回，`cerr` 输出 `[ERROR]` | `if (socket_fd < 0) { cerr << "[ERROR] ..."; return -1; }` |
| 行尾 | 使用 `std::endl` | 非 `\n` |
| 延迟 | `std::this_thread::sleep_for(std::chrono::milliseconds(n))` | — |
| 花括号 | 文件级函数换行，`main` 同行 | 见下文示例 |

```cpp
// 标准函数格式（文件级函数花括号换行）
void enable_servo(const int socket_fd)
{
  // 2空格缩进（部分文件）或4空格缩进（main）
  int state = -1;
  get_servo_state(socket_fd, state);
  ...
}

// main 函数（花括号同行）
int main() {
    // 4空格缩进
    auto socket_fd = connect_robot(ip, port);
    if (socket_fd < 0) {
        std::cerr << "[ERROR] 连接失败，程序退出!" << std::endl;
        return -1;
    }
    ...
    return 0;
}
```

## API 结构与命名规范

### 目录结构

```
include/
├── c/                  # C 语言 API（函数名以 _c 结尾）
│   ├── interface/      # tl_c_interface.h, tl_c_queue_operate.h, tl_c_io.h, ...
│   └── parameter/      # tl_define.h, tl_interface_parameter.h
└── cpp/                # C++ API（推荐使用）
    ├── interface/      # tl_api.h(一站式包含), tl_interface.h, tl_io.h, tl_modbus.h, tl_queue_operate.h, tl_track.h, tl_dual_arm.h, tl_vfd_ctr.h, tl_craft_*.h, ...
    └── parameter/      # tl_define.h, tl_interface_parameter.h, tl_modbus_parameter.h, tl_io_parameter.h, tl_parameter.h, tl_craft_*_parameter.h, ...

lib/                    # 平台相关动态库和 Python 绑定
├── x86/                # Linux x86_64
│   ├── _tl_host.so     # 核心动态库
│   └── tl_interface.py # Python 绑定
└── arm64/              # Linux ARM64
    ├── _tl_host.so
    ├── libmath_wrapper.so
    ├── libmodbus_wrapper.so
    ├── libservoJ_wrapper.so
    ├── libtl_host.so
    └── tl_interface.py

docs/                   # 产品文档（.docx）
├── SDK说明文档-linux-arm64-cpp.docx
├── SDK说明文档-linux-arm64-py.docx
├── SDK说明文档-linux-x86-cpp.docx
└── SDK说明文档-linux-x86-py.docx

example/                # 示例程序
├── cpp/                # C++ 示例
│   ├── CMakeLists.txt
│   ├── test_connect_api.cpp
│   ├── test_info_query_api.cpp
│   ├── test_queue_motion_api.cpp
│   └── ...（更多 .cpp 示例）
├── py/                 # Python 示例（开发中）
└── build/              # CMake 构建输出
```

### 一站式包含

```cpp
#include "cpp/interface/tl_api.h"  // 包含所有 C++ 接口和参数
```

### 命名规范

| 类别 | 规范 | 示例 |
|---|---|---|
| 函数 | `snake_case` | `connect_robot`, `set_servo_state`, `robot_movej`, `job_create`, `modbus_read_holding_registers` |
| 结构体/类 | `PascalCase` | `MoveCmd`, `ToolParam`, `RobotDHParam`, `ModbusMasterParameter` |
| 枚举 | `PascalCase`，值 `UPPER_SNAKE` | `enum Result { SUCCESS, ... }`, `enum class PosType { data, PType, ... }` |
| 宏/常量 | `UPPER_SNAKE` | `EXPORT_API`, `const_robotNum` |
| 类型别名 | `UPPER_SNAKE` | `using SOCKETFD = int` |
| 头文件守卫 | `INCLUDE_路径_文件名_H_`（全大写） | `INCLUDE_CPP_INTERFACE_TL_INTERFACE_H_` |
| 文件命名 | `snake_case`（tl_前缀） | `tl_interface.h`, `tl_job_operate.h` |
| C API 后缀 | 函数名以 `_c` 结尾 | `connect_robot_c`, `queue_motion_set_status_c` |

### 注释规范

- **语言**: 简体中文
- **格式**: Doxygen 风格 `/** @brief ... @param ... @return ... */`
- 函数注释说明: 功能、参数含义、返回值、使用注意事项
- 复杂流程使用 `@note` / `@attention` / `@warning` 标注关键约束
- 示例代码内使用行注释 `//` 说明每一段的作用
- 结构体字段逐行注释说明含义和取值范围

```cpp
// 示例: 标准 Doxygen 中文注释
/**
 * @brief 设置伺服状态
 * @param state 0 停止 / 1 就绪
 * @warning 需确保系统无错误后再调用
 */
EXPORT_API Result set_servo_state(SOCKETFD socketFd, int state);
```

### `MoveCmd` 结构体关键字段

```cpp
struct MoveCmd {
    PosType targetPosType{PosType::data}; // 点位类型: data(数值) / PType(变量) / GPType(全局)
    std::vector<double> targetPosValue;   // 坐标数据 (14位: 前7本体 + 后7外部轴)
    std::string targetPosName;            // 变量名 (如 "GP0001")
    int coord{0};                         // 0=关节 1=直角 2=工具 3=用户
    double velocity{50};                  // J: (1,100]°/s  L: (1,1000]mm/s
    double acc{50}, dec{50};
    int pl{0};                           // 平滑参数 [0,5]
};
```

## 关键约定与注意事项

1. **非阻塞运动**: `robot_movej` / `robot_movel` 立即返回，需用 `sleep_for` 等待到位
2. **队列运动**: 先 `queue_motion_set_status_c(true)` 打开模式，`push_back` 填充，最后 `send_to_controller` 一次性下发
3. **清错后不能直接上电**: 必须先下电再上电（`clear_error` → `set_servo_poweroff` → `set_servo_state(1)` → `set_servo_poweron`）
4. **机器人模式**: 0=示教 1=远程 2=运行；不同接口在不同模式下可用
5. **伺服状态**: 0=停止 1=就绪 2=报警 3=运行
6. **运行状态**: 0=停止 1=暂停 2=运行
7. **坐标系**: 0=关节 1=直角 2=工具 3=用户
8. **IP 默认**: `192.168.1.13`
9. **坐标数据长度**: `get_global_position` 返回 14 位数组 `[0]=坐标系, [1]=单位, [2]=形态, [3]=工具号, [4]=用户坐标, [5-6]=备用, [7-13]=坐标值`

## Python 绑定

- 文件: `lib/x86/tl_interface.py`（自动生成，**禁止手动修改**）
- 生成工具: SWIG 4.2.1
- 底层模块: `_tl_host`（需 `import _tl_host`，对应 `lib/x86/_tl_host.so`）

## 已知问题

- 无 CI、无 linter/formatter 配置、无单元测试
