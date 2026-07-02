# TL Robot Arm SDK — Linux/x86_64 C++

> 目录：[概述](#项目概述) · [快速开始](#快速开始) · [关键架构](#关键架构) · [编码规范](#编码规范) · [关键约定](#关键约定与注意事项) · [Python 绑定](#python-绑定) · [目录结构](#目录结构)

## 项目概述

机械臂控制 SDK，提供 C++ 和 C 双语言 API，通过 TCP/IP 远程控制机器人控制器。Python 绑定由 SWIG 自动生成。

**思考与输出语言**: 所有思考过程和输出必须使用中文。

---

## 快速开始

### 编译 C++ 示例

```bash
cd example/cpp/build
cmake ..
make
```

- 自动扫描 `example/cpp/` 下所有 `.cpp` 源文件，可执行文件名对应源文件名（如 `test_servo_api.cpp` → `test_servo_api`）
- 自动嵌入 RPATH，运行时无需设置 `LD_LIBRARY_PATH`

### 运行前准备

```bash
export LD_LIBRARY_PATH=lib/linux/x86:$LD_LIBRARY_PATH
example/cpp/build/test_connect_api
```

### 运行 Python 示例

```bash
cd example/py
python3.10 test_connect_api.py
```

---

## 关键架构

### 双端口连接模式

| 端口 | 用途 | 连接函数 |
|---|---|---|
| 6001 | 控制/状态（运动指令、IO、作业、Modbus） | `connect_robot(ip, "6001")` |
| 7000 | 伺服高频模式（servoJ 实时跟踪） | `connect_robot(ip, "7000")` |

- **6001** → 绝大多数接口使用
- **7000** → 仅用于 `open_servoJ`/`set_servoJ_pos`/`close_servoJ`（10ms 周期高频透传）
- **断开顺序**: `set_servo_poweroff` → `disconnect_robot`（下电可能自动断连，`disconnect_robot` 返回 -1 也视为成功）

### 上电流程（必须两步）

```cpp
set_servo_state(sock, 1);          // 步骤1: 伺服就绪
sleep_for(milliseconds(500));
set_servo_poweron(sock);           // 步骤2: 上电使能
```

### 错误码

| 值 | 含义 |
|---|---|
| 0 (SUCCESS) | 成功 |
| -1 (RECEIVE_FAILED) | 接收失败 |
| -2 (DISCONNECT) | 未连接 |
| -3 (PARAM_ERR) | 参数错误 |
| -4 (OPERATION_NOT_ALLOWED) | 操作不允许（如重复上电/下电） |
| -5 (EXCEPTION) | 异常 |
| -6 (TIMEOUT) | 超时 |

---

## 编码规范

### 代码风格

| 项目 | 规范 | 示例 |
|---|---|---|
| 头文件包含顺序 | 标准库 → 项目头文件 | `#include <iostream>` → `#include "cpp/interface/tl_api.h"` |
| 函数命名 | `snake_case` | `connect_robot`, `set_servo_state`, `robot_movej` |
| 结构体/类命名 | `PascalCase` | `MoveCmd`, `ToolParam`, `RobotDHParam` |
| 枚举命名 | `PascalCase`，值 `UPPER_SNAKE` | `enum Result { SUCCESS, ... }` |
| 宏/常量 | `UPPER_SNAKE` | `EXPORT_API`, `const_robotNum` |
| 类型别名 | `UPPER_SNAKE` | `using SOCKETFD = int` |
| 变量命名 | `snake_case` 优先 | `socket_fd`（推荐），`axisNum`（偶见） |
| 返回类型 | `auto` + `Result` 比较 | `auto ret = ...; if (ret != Result::SUCCESS)` |
| 错误处理 | 早期返回，cerr 输出 `[ERROR]` | `if (socket_fd < 0) { cerr << "[ERROR] ..."; return -1; }` |
| 行尾 | `std::endl` | 非 `\n` |
| 延迟 | `std::this_thread::sleep_for(...)` | — |
| 花括号 | 文件级函数换行，`main` 同行 | 见下文 |
| 头文件守卫 | `INCLUDE_路径_文件名_H_`（全大写） | `INCLUDE_CPP_INTERFACE_TL_INTERFACE_H_` |
| C API 后缀 | 函数名 `_c` 结尾 | `connect_robot_c`, `queue_motion_set_status_c` |

```cpp
// 文件级函数花括号换行
void enable_servo(const int socket_fd)
{
    int state = -1;
    get_servo_state(socket_fd, state);
    ...
}

// main 花括号同行
int main() {
    auto socket_fd = connect_robot(ip, port);
    if (socket_fd < 0) {
        std::cerr << "[ERROR] 连接失败，程序退出!" << std::endl;
        return -1;
    }
    ...
    return 0;
}
```

### 注释规范

- **语言**: 简体中文全文统一
- **文件头**: Doxygen `/** ... */` 含 `@brief`、`@attention`、`@note`
- **函数签名**: `/** @brief ... @param ... @return ... @warning ... */`
- **行内注释**: `//` 中文说明
- **节标题**: `// ----功能描述----` 用 `----` 包裹
- **结构体字段**: 逐行行尾注释说明含义和取值范围
- **日志前缀**: `[INFO]` 正常信息，`[ERROR]` 错误信息
- **复杂约束**: 使用 `@note` / `@attention` / `@warning` 标注

```cpp
/**
 * @brief 设置伺服状态
 * @param state 0 停止 / 1 就绪
 * @warning 需确保系统无错误后再调用
 */
EXPORT_API Result set_servo_state(SOCKETFD socketFd, int state);
```

```cpp
// 文件头示例
/**
 * filename.cpp
 * @brief 一句话描述功能
 * @attention 运行前提条件/注意事项
 * @note 运行步骤
 *       编译: g++ -std=c++11 filename.cpp -o output -I./include ./lib/linux/x86/libtl_host.so -lpthread
 *       链接动态库: export LD_LIBRARY_PATH=lib/linux/x86:$LD_LIBRARY_PATH
 *       运行: ./output
 */
```

---

## 关键约定与注意事项

### 数值常量速查

| 概念 | 值 | 说明 |
|---|---|---|
| 机器人模式 | 0=示教 1=远程 2=运行 | 不同接口在不同模式下可用 |
| 伺服状态 | 0=停止 1=就绪 2=报警 3=运行 | — |
| 运行状态 | 0=停止 1=暂停 2=运行 | — |
| 坐标系 | 0=关节 1=直角 2=工具 3=用户 | — |
| 默认 IP | `192.168.1.13` | — |
| 坐标数据 | 14 位数组 | `[0]=坐标系, [1]=单位, [2]=形态, [3]=工具号, [4]=用户坐标, [5-6]=备用, [7-13]=坐标值` |

### 运动控制要点

1. **非阻塞运动**: `robot_movej` / `robot_movel` 立即返回，需用 `sleep_for` 等待到位
2. **队列运动**: `queue_motion_set_status_c(true)` → `push_back` 填充 → `send_to_controller` 一次性下发
3. **清错后恢复**: 必须先下电再上电 — `clear_error` → `set_servo_poweroff` → `set_servo_state(1)` → `set_servo_poweron`
4. **servoJ 断开**: `close_servoJ` → 下电 → 断开（下电可能自动断连，返回 -1 视为成功）

### servoJ vector 长度约定

SDK 底层协议统一使用 **7 元素 vector** 兼容 6 轴和 7 轴机械臂：

- **6 轴机器人**: 第 7 元素（索引 6）补 0（外部轴）
- **7 轴机器人**: 第 7 元素是真实 J7 关节角

涉及接口：`open_servoJ` 的 `vmax/amax/jmax`、`set_servoJ_pos` 的 `q`。

**编码要点**: 不要硬编码 `range(6)` + `q[6]=0`。应根据实际轴数动态写入，剩余元素保持 0（vector 初始化即置零）。

### 已知坑

5. **SWIG bool& 参数重载 bug**（x86 Python 绑定）: `get_origin_coord_to_target_coord` 在 SWIG 4.2.1 生成的绑定中，因重载函数包含 `bool&` 类型参数，调用时会抛出 `TypeError`。所有调用必须用 `try/except TypeError` 包装。（ARM64 ctypes 绑定无此问题）
6. **Python 绑定平台差异**: x86 和 Windows 使用 SWIG 生成（`_tl_host.so`/`tl_host.pyd`），ARM64 使用 ctypes 生成（`libservoJ_wrapper.so` + `libmath_wrapper.so` + `libmodbus_wrapper.so`）。不同平台下 `set_servoJ_pos` 等接口的实现路径不同，但 Python API 签名一致。

### `MoveCmd` 结构体关键字段

```cpp
struct MoveCmd {
    PosType targetPosType{PosType::data}; // 点位类型: data(数值) / PType(变量) / GPType(全局)
    std::vector<double> targetPosValue;   // 坐标数据 (14位: 前7本体 + 后7外部轴)
    std::string targetPosName;            // 变量名 (如 "GP0001")
    int coord{0};                         // 0=关节 1=直角 2=工具 3=用户
    double velocity{50};                  // J: (1,100]°/s  L: (1,1000]mm/s
    double acc{50}, dec{50};
    int pl{0};                            // 平滑参数 [0,5]
};
```

---

## Python 绑定

- **文件**: `lib/linux/x86/python/tl_interface.py`（SWIG 自动生成，Linux x86，**禁止手动修改**）
- **文件**: `lib/linux/arm64/python/tl_interface.py`（ctypes 生成，Linux ARM64）
- **文件**: `lib/windows/python/tl_interface.py`（SWIG 自动生成，Windows）
- **生成工具**: SWIG 4.2.1（x86 / Windows），ctypes（ARM64）
- **Python 版本**: 扩展模块编译目标为 **Python 3.10**，请使用 Python 3.10 运行
- **绑定入口**: `from tl_interface import *`（需先 `sys.path.append(".../python")`）

---

## 目录结构

```
include/               # C++ 和 C 头文件
├── c/                 # C API（函数名 _c 结尾）
└── cpp/               # C++ API（推荐，tl_api.h 一站式包含）
lib/                   # 平台动态库 + Python 绑定
├── linux/x86/         # Linux x86_64 (libtl_host.so + SWIG python)
├── linux/arm64/       # Linux ARM64 (libtl_host.so + ctypes python)
└── windows/           # Windows x86_64 (tl_host.dll + SWIG python)
docs/                  # 产品文档 (.docx)
example/               # 示例程序
├── cpp/               # C++ 示例 (CMakeLists.txt 自动构建)
├── py/                # Python 示例 (python3.10 直接运行)
└── build/             # CMake 构建输出
```

```cpp
// 一站式包含
#include "cpp/interface/tl_api.h"
```

---

## 已知问题

- 无 CI、无 linter/formatter 配置、无单元测试
