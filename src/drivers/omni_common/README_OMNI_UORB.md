# Omni 矢量四旋翼 uORB 话题记录（当前代码）

本文记录 Omni 矢量四旋翼开发中，**当前实际使用并发布** 的 uORB 话题。

## 编译开关

- 共享开关文件：[omni_debug.h](/home/jay-jie/PX4-Autopilot/src/drivers/omni_common/omni_debug.h)
- `OMNI_DEBUG_SINGLE` 用于切换“单执行器调试链路”和“四执行器分组链路”。
- 目前文件状态为 `// #define OMNI_DEBUG_SINGLE`（注释状态），默认走分组链路。

## 已发布话题

| 话题名 | 消息类型 | 发布模块 | 条件/说明 |
|---|---|---|---|
| `omni_actuator_setpoint` | `OmniActuatorSetpoint.msg` | `ActuatorEffectivenessOmniMultirotor` | 控制分配效能更新路径发布。见 [ActuatorEffectivenessOmniMultirotor.cpp:139](/home/jay-jie/PX4-Autopilot/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessOmniMultirotor.cpp:139)。 |
| `omni_outputs_cmd` | `OmniOutputsCmd.msg` | `omni_swashplateless` | 单执行器调试命令输出（`OMNI_DEBUG_SINGLE` 路径）。见 [OmniSwashplateless.cpp:183](/home/jay-jie/PX4-Autopilot/src/drivers/omni_swashplateless/OmniSwashplateless.cpp:183)。 |
| `omni_outputs_cmd_param` | `OmniOutputsCmdParam.msg` | `omni_swashplateless` | 调试参数轨迹输出（`OMNI_DEBUG_SINGLE` 路径）。见 [OmniSwashplateless.cpp:664](/home/jay-jie/PX4-Autopilot/src/drivers/omni_swashplateless/OmniSwashplateless.cpp:664)。 |
| `omni_outputs_cmd_groups` | `OmniOutputsCmdGroups.msg` | `omni_swashplateless` | 四执行器分组命令输出（`#ifndef OMNI_DEBUG_SINGLE` 路径）。见 [OmniSwashplateless.cpp:143](/home/jay-jie/PX4-Autopilot/src/drivers/omni_swashplateless/OmniSwashplateless.cpp:143)。 |
| `omni_outputs_cmd_frame` | `OmniOutputsCmdFrame.msg` | `omni_uart_io` | 串口发送子包打包后发布（单包和分组发送都会经过该路径）。见 [omni_uart_io.cpp:446](/home/jay-jie/PX4-Autopilot/src/drivers/omni_uart_io/omni_uart_io.cpp:446)。 |
| `omni_motor_telemetry` | `OmniMotorTelemetry.msg` | `omni_uart_io` | 单包接收遥测发布（`OMNI_DEBUG_SINGLE` 接收路径）。见 [omni_uart_io.cpp:518](/home/jay-jie/PX4-Autopilot/src/drivers/omni_uart_io/omni_uart_io.cpp:518)。 |
| `omni_motors_telemetry` | `OmniMotorsTelemetry.msg` | `omni_uart_io` | 四电机分组接收遥测发布（`#ifndef OMNI_DEBUG_SINGLE` 接收路径）。见 [omni_uart_io.cpp:280](/home/jay-jie/PX4-Autopilot/src/drivers/omni_uart_io/omni_uart_io.cpp:280)。 |

## 主链路流程

### 1) 分组链路（未定义 `OMNI_DEBUG_SINGLE`）

1. 控制分配发布 `omni_actuator_setpoint`。
2. `omni_swashplateless` 计算并发布 `omni_outputs_cmd_groups`。
3. `omni_uart_io` 订阅 `omni_outputs_cmd_groups`，发送四包合并 UART 协议（外层大包 + 4 个子包）。
4. `omni_uart_io` 接收合并反馈包，发布 `omni_motors_telemetry`。

### 2) 单执行器调试链路（定义 `OMNI_DEBUG_SINGLE`）

1. `omni_swashplateless` 发布 `omni_outputs_cmd` 与 `omni_outputs_cmd_param`。
2. `omni_uart_io` 订阅 `omni_outputs_cmd`，发送单子包 UART 协议。
3. `omni_uart_io` 接收单包反馈，发布 `omni_motor_telemetry`。

## 关键声明位置（Pub/Sub 成员）

- `omni_swashplateless`：[OmniSwashplateless.h:98](/home/jay-jie/PX4-Autopilot/src/drivers/omni_swashplateless/OmniSwashplateless.h:98)
- `omni_uart_io`：[omni_uart_io.hpp:169](/home/jay-jie/PX4-Autopilot/src/drivers/omni_uart_io/omni_uart_io.hpp:169)
- `ActuatorEffectivenessOmniMultirotor`：[ActuatorEffectivenessOmniMultirotor.hpp:64](/home/jay-jie/PX4-Autopilot/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessOmniMultirotor.hpp:64)
