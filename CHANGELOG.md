# Changelog

## V20260629

**SDK 版本**: 2.0.4

### API 新增

#### 通用接口（`tl_interface.h` / `tl_c_interface.h`）

| 函数 | 说明 |
|---|---|
| `set_connect_timeout_seconds` / `_c` | 设置连接超时时间，超时后直接返回错误 |
| `get_full_solution` | 获取运动学完整解 |
| `get_end_tool_state_gi` / `_c` | 获取末端工具状态（通用接口） |
| `set_end_tool_state_gi` / `_c` | 设置末端工具状态（通用接口） |
| `get_independent_axis_pv_and_pp_mode_max_vel` / `_c` | 获取独立轴 PV/PP 模式最大速度 |
| `set_independent_axis_pv_and_pp_mode_max_vel` / `_c` | 设置独立轴 PV/PP 模式最大速度 |

#### 力传感器 — 作业指令（`tl_job_operate.h`）

| 函数 | 说明 |
|---|---|
| `job_insert_force_collision_off` / `_robot` | 插入关闭力碰撞检测指令 |
| `job_insert_force_collision_on` / `_robot` | 插入开启力碰撞检测指令 |
| `job_insert_force_payload_add` / `_robot` | 插入添加力负载指令 |
| `job_insert_force_payload_remove` / `_robot` | 插入移除力负载指令 |
| `job_insert_force_threshold_set` / `_robot` | 插入设置力阈值指令 |
| `job_insert_axis_motion_speed_limit` / `_robot` | 插入轴运动速度限制指令 |

#### 力传感器 — 队列运动（`tl_queue_operate.h`）

| 函数 | 说明 |
|---|---|
| `queue_motion_push_back_force_collision_off` / `_robot` | 队列插入关闭力碰撞检测 |
| `queue_motion_push_back_force_collision_on` / `_robot` | 队列插入开启力碰撞检测 |
| `queue_motion_push_back_force_payload_add` / `_robot` | 队列插入添加力负载 |
| `queue_motion_push_back_force_payload_remove` / `_robot` | 队列插入移除力负载 |
| `queue_motion_push_back_force_threshold_set` / `_robot` | 队列插入设置力阈值 |
| `queue_motion_push_back_axis_motion_speed_limit` / `_robot` | 队列插入轴速度限制 |

#### 队列运动扩展（`tl_queue_operate.h`）

| 函数 | 说明 |
|---|---|
| `queue_motion_push_back_moveC_extra` / `_robot` | 队列插入圆弧辅助点运动 |

### API 参数变更

| 函数 | 变更 |
|---|---|
| `get_origin_coord_to_target_coord` / `_robot` | 新增出参 `bool& convert_state`，指示坐标转换是否成功 |

### 新增结构体

| 结构体 | 头文件 | 说明 |
|---|---|---|
| `AxisSpeedLimit` | `tl_define.h` | 轴运动速度限制参数 |
| `InverseKinParameter` | `tl_interface_parameter.h` | 逆解参数 |

### 平台支持扩展

| 平台 | V20260529 | V20260629 |
|---|---|---|
| Linux x86_64 | ✅ `lib/x86/` | ✅ `lib/linux/x86/`（目录重组） |
| Linux ARM64 | ✅ `lib/arm64/` | ✅ `lib/linux/arm64/`（目录重组） |
| **Windows x86_64** | ❌ 不支持 | ✅ 新增 |

### 目录结构调整

库目录按 `lib/{系统}/{架构}/` 标准化组织：

| 旧路径 | 新路径 |
|---|---|
| `lib/x86/_tl_host.so` | `lib/linux/x86/libtl_host.so` |
| `lib/x86/tl_interface.py` | `lib/linux/x86/python/tl_interface.py` |
| `lib/arm64/*` | `lib/linux/arm64/` + `python/` 子目录 |
| — | `lib/windows/tl_host.dll`（新增） |
| — | `lib/windows/python/tl_host.pyd`（新增） |

### Python 绑定

- Python 绑定文件统一移至 `<platform>/python/` 子目录
- Windows 新增 SWIG 生成的 `.pyd` 扩展模块
- 明确 Python 3.10 版本要求（SWIG 扩展编译目标）
- Linux x86 SWIG 模块内部以 `_nrc_host` 模块名加载（兼容 `PyInit__nrc_host`）

### 文档

- `README.md`：目录结构、编译路径、Python 版本要求更新
- `AGENTS.md`：目录结构、编译路径、Python 绑定说明更新
- `CHANGELOG.md`：新增

---

## V20260529

初始版本，Taurus 机器人 SDK v2.0.4。

- Linux x86_64 + ARM64 支持
- C/C++ API（运动控制、IO、作业管理、Modbus、伺服跟踪、队列运动、力传感器数据读取等）
- Python 绑定（SWIG 生成）
