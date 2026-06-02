# MPC_POS_BACKEND 使用说明

## 结论

`MPC_POS_BACKEND` 用来选择多旋翼位置控制器输出后端。

- `0: Legacy`：标准 PX4 四旋翼/多旋翼模式。
- `1: Omni3D`：Omni 机体使用的 3D 推力输出模式。

运行 `HEADLESS=1 make px4_sitl jmavsim` 调试 jMAVSim Iris 四旋翼时，应使用：

```sh
param set MPC_POS_BACKEND 0
param save
```

如果 `MPC_POS_BACKEND=1`，位置控制器会按 Omni3D 方式输出横向 body thrust，标准四旋翼无法直接产生机体系横向力，前飞或定点时可能表现为漂移、响应异常或不能按传统倾斜方式前进。

## 调用链

入口函数：

```text
src/modules/mc_pos_control/MulticopterPositionControl.cpp
```

参数更新路径：

```text
MulticopterPositionControl::parameters_update()
  -> _param_mpc_pos_backend.get()
  -> PositionControl::setOutputBackend()
```

位置控制输出路径：

```text
MulticopterPositionControl::Run()
  -> PositionControl::update()
  -> PositionControl::getAttitudeSetpoint(attitude_setpoint, vehicle_attitude)
  -> publish vehicle_attitude_setpoint
```

后端分支：

```text
PositionControlBackend::Legacy
  -> ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint)

PositionControlBackend::Omni3D
  -> R_world_to_body * _thr_sp
  -> attitude_setpoint.thrust_body[]
  -> q_d 使用期望姿态或当前姿态
```

## 参数说明

### MPC_POS_BACKEND

定义位置：

```text
src/modules/mc_pos_control/multicopter_position_control_params.c
```

含义：

- 选择多旋翼位置控制器输出后端。
- 默认值是 `0`。
- 这是运行时参数，参数保存后会覆盖 airframe 中的默认值。

取值：

| 值 | 名称 | 用途 |
| --- | --- | --- |
| `0` | `Legacy` | 标准四旋翼/多旋翼。位置控制器把期望推力向量转换为姿态 setpoint，通过倾斜机体产生水平加速度。 |
| `1` | `Omni3D` | Omni 机体。位置控制器把世界系推力转换到机体系 `thrust_body`，用于 6DoF 或可直接产生横向力的执行器系统。 |

### CA_AIRFRAME

定义位置：

```text
src/modules/control_allocator/module.yaml
```

关键取值：

| 值 | 名称 | 用途 |
| --- | --- | --- |
| `0` | `Multirotor` | 标准多旋翼 effectiveness。jMAVSim Iris 应使用这个值。 |
| `16` | `Omni_Multirotor` | Omni 多旋翼 effectiveness，会创建 `ActuatorEffectivenessOmniMultirotor`。 |

jMAVSim Iris airframe：

```text
ROMFS/px4fmu_common/init.d-posix/airframes/10017_jmavsim_iris
```

其中默认：

```sh
param set-default CA_AIRFRAME 0
```

注意：`param set-default` 只设置默认值，不会覆盖已保存参数。如果 `parameters.bson` 中已经保存了其他值，需要手动 `param reset` 或 `param set`。

### CA_ROTOR_COUNT / CA_ROTORx_*

jMAVSim Iris 默认四旋翼几何参数在 airframe 文件中设置：

```sh
param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX ...
param set-default CA_ROTOR0_PY ...
param set-default CA_ROTOR0_KM ...
```

这些参数只影响 control allocator 的 multirotor effectiveness 矩阵。调试标准四旋翼时，不应为了 Omni 机体修改 Iris 的 rotor 几何。

### PWM_MAIN_FUNCx

jMAVSim Iris 默认输出映射：

```sh
param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104
```

含义是主输出 1-4 分别映射到 Motor 1-4。标准 jMAVSim 四旋翼应保持这组映射。

## 使用方法

### 标准 jMAVSim 四旋翼

适用命令：

```sh
HEADLESS=1 make px4_sitl jmavsim
```

推荐参数：

```sh
param set SYS_AUTOSTART 10017
param set CA_AIRFRAME 0
param set MPC_POS_BACKEND 0
param save
```

检查命令：

```sh
param show SYS_AUTOSTART
param show CA_AIRFRAME
param show MPC_POS_BACKEND
param show CA_ROTOR_COUNT
```

期望结果：

```text
SYS_AUTOSTART = 10017
CA_AIRFRAME = 0
MPC_POS_BACKEND = 0
CA_ROTOR_COUNT = 4
```

### Omni3D 机体

Omni3D 模式只适用于执行器系统能够直接产生机体系横向力的机体。

推荐参数组合：

```sh
param set MPC_POS_BACKEND 1
param set CA_AIRFRAME 16
param save
```

需要同时确认：

- `ActuatorEffectivenessOmniMultirotor` 是否被选中。
- `omni_actuator_setpoint` 是否被发布。
- `omni_swashplateless` 是否订阅并转换为 `omni_outputs_cmd_groups`。
- `omni_uart_io` 是否将 `omni_outputs_cmd_groups` 正确打包发送到外部执行器。

## 常见问题

### jMAVSim 前飞时像在漂移

优先检查：

```sh
param show MPC_POS_BACKEND
```

如果是：

```text
MPC_POS_BACKEND = 1
```

切回：

```sh
param set MPC_POS_BACKEND 0
param save
```

原因是标准四旋翼需要 `Legacy` 路径，通过 `thrustToAttitude()` 把水平加速度需求转换为俯仰/横滚姿态。`Omni3D` 路径会尝试输出 body X/Y thrust，jMAVSim Iris 的四个竖直旋翼无法执行这个控制量。

### airframe 默认值没有生效

PX4 参数有持久化机制。已经保存到 `parameters.bson` 的参数优先级高于 airframe 中的 `param set-default`。

处理方法：

```sh
param reset MPC_POS_BACKEND
param reset CA_AIRFRAME
param save
```

或者直接设置期望值：

```sh
param set MPC_POS_BACKEND 0
param set CA_AIRFRAME 0
param save
```

## 注意事项

- 标准四旋翼调试时，`MPC_POS_BACKEND` 必须是 `0`。
- Omni3D 调试时，`MPC_POS_BACKEND=1` 还不够，还需要 `CA_AIRFRAME=16` 和完整的 omni 执行器链路。
- 不要只看 airframe 文件里的 `param set-default`，要用 `param show` 查看运行时真实参数。
- 修改 `MPC_POS_BACKEND` 后建议重启 SITL，避免不同模块处在旧状态。
- 如果 `rcS` 自动启动了 `omni_uart_io` 和 `omni_swashplateless`，调试 jMAVSim 四旋翼时要确认它们没有干扰判断。
- 修改位置控制、control allocation 或执行器输出后，建议至少重新编译并运行一次 SITL 验证。

## 快速恢复四旋翼 SITL

```sh
param set SYS_AUTOSTART 10017
param set CA_AIRFRAME 0
param set MPC_POS_BACKEND 0
param set CA_ROTOR_COUNT 4
param save
shutdown
```

重新启动：

```sh
HEADLESS=1 make px4_sitl jmavsim
```
