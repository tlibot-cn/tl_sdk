# Python 示例程序

SWIG 生成的 `tl_interface.py` 提供了与 C++ 一致的 API。示例无需编译，直接运行即可。

---

## 环境要求

- **Python 3.10**（扩展模块编译目标为此版本）

## 运行

```bash
cd example/py
python3.10 test_connect_api.py
```

也可在项目根目录直接运行：

```bash
python3.10 example/py/test_connect_api.py
```

## 示例一览

| 示例 | 核心功能 | 适合谁看 |
|---|---|---|
| `test_connect_api.py` | 连接控制器 → 获取 SDK 版本 → 断开 | 初次接触，验证通信是否正常 |
| `test_info_query_api.py` | 查询位置、运行状态、关节角度、DH 参数 | 需要读取机器人信息时参考 |
| `test_servo_api.py` | 伺服状态管理：就绪→上电→运行→下电→停止，速度查询/设置 | 了解伺服状态机与速度控制 |
| `test_move_control_api.py` | MoveJ（关节运动）、MoveL（直线运动） | 基础运动控制 |
| `test_queue_motion_api.py` | 队列运动完整流程：开启模式 → 编排 MoveJ/MoveL → 下发执行 | 需要预编排多条轨迹时参考 |
| `test_servoj_api.py` | servoJ 关节实时跟踪（7000 端口高频透传，10ms 周期） | 需要实时关节控制时参考 |
| `test_digital_io_api.py` | 数字 IO 输出/输入读写，模式切换 | IO 控制 |
| `test_modbus_api.py` | Modbus TCP 主站配置、写保持寄存器、读保持寄存器 | 需要通过 Modbus 通信时参考 |
| `test_job_control_api.py` | 创建作业 → 插入 MoveJ/L/C → 获取作业列表 → 运行作业 | 作业文件管理 |
| `test_coord_tool_api.py` | 工具手参数、用户坐标系、坐标编号、当前坐标系设置 | 坐标系与工具管理 |
| `test_coord_transform_api.py` | RPY ↔ 四元数 ↔ 旋转矩阵，TR ↔ R，正逆解 + servoJ 运动 | 坐标变换算法 |
| `test_error_callback_api.py` | 注册错误/警告回调 → 通过逆解不可达点触发回调 | 异常处理与消息监听 |
| `test_error_zero_api.py` | 清除错误、设置指定关节零位 | 错误恢复与零位标定 |
| `test_global_position_api.py` | 设置/查询全局路点 | 全局路点管理 |
| `test_log_download_api.py` | 从控制器下载日志文件到本地 | 需要诊断/调试时参考 |
| `test_teach_control_api.py` | 切换示教模式 → 关节点动 | 手动操作与示教 |
| `test_six_axis_force_api.py` | 六维力传感器通讯参数设置与数据读取 | 需要力传感器数据时参考 |

---

## 安全注意事项

以下接口涉及机器人标定、网络配置，误用可能导致断连、定位偏差、机械碰撞或设备损坏：

| 接口 | 风险 | 示例 | 说明 |
|---|---|---|---|
| `set_axis_zero_position` / `set_sync_axis_zero_position` | **极高** — 修改关节零位标定，所有运动基准偏移 | `test_error_zero_api.py` | 默认已注释，仅出厂校准或更换关节后使用 |
| `set_zero_pos_deviation` | **极高** — 修改零位偏差值，直接改变关节零点 | — | 与零位标定同等危险 |
| `set_robot_dh_param` | **极高** — 修改 DH 参数，运动学解算完全错误，可能导致不可预测运动 | — | 仅在更换连杆或重新标定后使用 |
| `set_robot_joint_param` | **极高** — 修改关节参数（限位、减速比等），可能导致超限或碰撞 | — | 确认参数来源，错误值会导致物理损坏 |
| `set_controller_ip` / `set_controller_network_config` | **极高** — 修改控制器 IP 或网络配置，立即断连且可能无法重新连接 | — | 需通过其他方式恢复网络时极为麻烦 |
| `set_result_for_DH` | **极高** — 应用 DH 标定结果，错误标定导致运动学完全混乱 | — | 仅在完整的 DH 标定流程后调用 |
| `set_collision_para` | **高** — 碰撞检测参数设得过大则碰撞不检测，设得过小则频繁误触发 | — | 在安全前提下通过实验调整 |
| `set_tool_hand_param` | **中** — 工具参数影响 TCP 位姿计算 | `test_coord_tool_api.py` | 确认工具实际尺寸后再设置 |
| `set_tool_coordinate_range` | **中** — 工具坐标范围限制，错误值限制关节运动 | — | 确认工具可达范围后再设置 |
| `set_user_coordinate_data` | **中** — 用户坐标系影响路径规划 | `test_coord_tool_api.py` | 确认坐标系定义后再设置 |
| `set_position_dragParams` / `set_darg_mode` | **中** — 拖动示教参数，错误值导致拖动过重或过轻 | — | 根据实际负载和工具配置 |
| `set_four_point_mark` | **中** — 四点标定标记，错误标记影响工具/用户坐标标定 | — | 在完整标定流程中按步骤使用 |
| `set_global_position` / `set_global_sync_position` | **低** — 修改全局路点数据，错误路点导致运动到意外位置 | `test_global_position_api.py` | 确认路点值在安全工作范围内 |
| `set_reconnect` | **低** — 重连机制，网络异常时行为可能不符合预期 | — | 确认网络策略后再启用 |

> **通用原则**: 在真实机械臂上运行任何示例前，请先确认工作区域安全、速度设置在合理范围内，并随时准备按下急停按钮。

## 已知限制

x86 SWIG 绑定（`_tl_host.so`）中 `get_origin_coord_to_target_coord` 因 SWIG 4.2.1 重载函数的 `bool&` 参数解析存在兼容性问题，相关调用已用 `try/except TypeError` 包装。

ARM64 ctypes 绑定无此问题。
