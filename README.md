# TL Robot Arm SDK

通过 **C++** / **C** 语言远程控制机器人控制器的 SDK。基于 TCP/IP 通信，提供运动控制、IO 控制、作业管理、Modbus 等完整接口。Python 绑定由 SWIG 自动生成。

---

## SDK 能做什么

- **关节运动**（MoveJ）和 **直线运动**（MoveL）—— 发送目标位置，非阻塞执行
- **伺服实时跟踪**（servoJ）—— 10ms 周期高频透传，实时控制每个关节角度
- **队列运动** —— 预编排多条轨迹，一次性下发执行
- **IO 控制** —— 读写数字/模拟输入输出
- **作业管理** —— 创建、编辑、运行机器人作业文件
- **Modbus 通信** —— 配置主站，读写外部设备寄存器
- **信息查询** —— 当前位置、伺服状态、运行状态、DH 参数、关节温度等
- **Python 调用** —— 通过 SWIG 生成的 `tl_interface.py` 直接调用

---

## 示例程序

- C++ 示例 → [`example/cpp/README.md`](example/cpp/README.md)
- Python 示例 → [`example/py/README.md`](example/py/README.md)

所有示例覆盖连接控制、运动控制、队列运动、servoJ 实时跟踪、IO、Modbus、作业管理、坐标变换、错误处理等功能。

---

## 在自己的项目中使用 SDK

### 目录结构

SDK 的核心文件分布在以下目录：

```
your_project/
├── include/                 # C++ 和 C 头文件
│   ├── c/                   # C API 头文件
│   │   ├── interface/       #   C 接口声明
│   │   └── parameter/       #   C 参数定义
│   └── cpp/                 # C++ API 头文件（推荐）
│       ├── interface/       #   C++ 接口声明（tl_api.h 一站式包含）
│       └── parameter/       #   C++ 参数定义
├── lib/                     # 平台相关动态库
│   ├── linux/
│   │   ├── x86/             # Linux x86_64
│   │   │   ├── libtl_host.so        # 核心动态库（链接用）
│   │   │   └── python/
│   │   │       ├── _tl_host.so      # Python 扩展模块（SWIG）
│   │   │       └── tl_interface.py  # Python 绑定入口
│   │   └── arm64/           # Linux ARM64
│   │       ├── libtl_host.so
│   │       ├── libmath_wrapper.so
│   │       └── python/
│   │           ├── libtl_host.so    # ctypes 加载的动态库
│   │           └── tl_interface.py  # Python 绑定入口（ctypes）
│   └── windows/             # Windows x86_64
│       ├── tl_host.dll              # 核心动态库
│       ├── tl_host.lib              # 导入库
│       └── python/
│           ├── tl_host.pyd          # Python 扩展模块（SWIG）
│           └── tl_interface.py      # Python 绑定入口
├── docs/                    # 产品文档（.docx）
└── example/                 # 示例程序
    ├── cpp/                 # C++ 示例源码 + CMakeLists.txt
    └── py/                  # Python 示例
```

### C++ 项目

`#include` 只需一行：

```cpp
#include "cpp/interface/tl_api.h"  // 包含所有 C++ API
```

编译时链接 `libtl_host.so` 和 `-lpthread`：

```bash
g++ -std=c++11 my_program.cpp -o my_program -I./include ./lib/linux/x86/libtl_host.so -lpthread
export LD_LIBRARY_PATH=lib/linux/x86:$LD_LIBRARY_PATH
./my_program
```

### Python 项目

```python
import sys
sys.path.append("lib/linux/x86/python")
from tl_interface import *
sock = connect_robot("192.168.1.13", "6001")
```

> **⚠️ Python 版本要求**
> `.pyd`/`.so` 扩展模块编译目标为 **Python 3.10**（依赖 `python310.dll` / `PyInit__nrc_host`），请使用 Python 3.10 运行。
>
> - Linux: `python3.10 lib/linux/x86/python/tl_interface.py`
> - Windows: `py -3.10 lib/windows/python\tl_interface.py`
>
> 其他版本 Python 需要自行用 SWIG 重新编译生成绑定。

---

## 核心概念

### 双端口连接

SDK 使用两个 TCP 端口分工：

- **6001 端口** —— 控制与状态。所有运动指令、IO、作业、Modbus 等常规接口都走此端口
- **7000 端口** —— 伺服高频模式。仅用于 `open_servoJ` / `set_servoJ_pos` / `close_servoJ`（10ms 周期透传）

如果只做常规运动控制，只需要连接 6001 端口。servoJ 实时跟踪才需要额外连接 7000 端口。

### 上电 / 下电

上电必须分两步走：

```cpp
set_servo_state(sock, 1);          // ① 伺服就绪
std::this_thread::sleep_for(std::chrono::milliseconds(500));
set_servo_poweron(sock);           // ② 上电使能
```

下电后网络会自动断开，此时 `disconnect_robot` 返回 -1 是正常现象，无需处理。

### 运动控制特点

- **非阻塞**：`robot_movej` / `robot_movel` 发出指令后立即返回，需要通过 `sleep_for` 等待机械臂实际到位
- **坐标系**：0=关节 1=直角 2=工具 3=用户
- **队列模式**：先 `queue_motion_set_status_c(true)` 开启，多次 `push_back` 添加轨迹点，最后 `send_to_controller` 一次性下发

### 清错后恢复

清除错误后不能直接上电，需要按顺序操作：

```
clear_error → set_servo_poweroff → set_servo_state(1) → set_servo_poweron
```

---

## 典型调用流程

```
连接控制器（6001 端口）
  ↓
清错
  ↓
伺服就绪 → 上电使能
  ↓
运动控制 / IO / 作业 / 查询 ...
  ↓
下电
  ↓
断开连接
```

---

## 注意事项

- **默认 IP**：`192.168.1.13`，请确保控制器网络可达
- `tl_interface.py` 由 SWIG 自动生成，请勿手动修改
- 部分接口仅在特定机器人模式下可用（示教/运行/远程），详见头文件注释
- 动态库位于 `lib/linux/x86/`（Linux x86_64）、`lib/linux/arm64/`（Linux ARM64）、`lib/windows/`（Windows）；CMake 构建已嵌入 RPATH 无需额外设置，手动 g++ 编译需设置 `LD_LIBRARY_PATH=lib/linux/x86`
- Python 绑定（`tl_interface.py`）由 SWIG 自动生成，需 Python 3.10 环境加载
