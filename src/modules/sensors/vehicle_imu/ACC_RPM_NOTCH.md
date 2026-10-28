# ACC RPM 动态陷波

本实现使用 `esc_status` 的机械转速更新 ACC 陷波频率，作用于 EKF 的惯性输入。
默认关闭，不需要 `gyro_fft`。不支持 `CONSTRAINED_FLASH` 构建。

## 数据路径与职责

`sensor_accel` → `VehicleIMU::UpdateAccel()` → `AccelRpmNotch` → ACC 积分 → 原有校准和机体系旋转 → `vehicle_imu` → 估计器。

- 输入为传感器坐标系三轴加速度，单位 m/s²；积分输出为速度增量，单位 m/s。
- `esc_status` 定义见 [EscStatus.msg](/home/jay-jie/PX4-Autopilot/msg/EscStatus.msg)，转速字段见 [EscReport.msg](/home/jay-jie/PX4-Autopilot/msg/EscReport.msg)。当前 Omni 发布者为 `omni_uart_io`，订阅者为各 `VehicleIMU` 实例。
- Omni 驱动每 5 ms 调度一次，收到有效四电机反馈帧后发布 `esc_status`；实际话题频率取决于反馈到达情况。
- 复用 PX4 `NotchFilter<Vector3f>`；每个电机每阶谐波使用同一组系数处理三轴，各轴历史独立。
- 每个 IMU 实例独立持有滤波器、转速缓存和采样时序状态。计算在该实例原有 WorkQueue 中执行，由 gyro 回调及原有备用调度触发，没有新增线程或定时器。
- 每次 Run 更新转速与超时状态；每个消费到的 ACC 样本执行滤波。
- 不更改 UART 协议、uORB 消息定义、gyro 控制反馈或 `vehicle_acceleration` 的独立低通路径。

## 参数

参数定义见 [imu_parameters.c](/home/jay-jie/PX4-Autopilot/src/modules/sensors/vehicle_imu/imu_parameters.c)。所有参数在 `VehicleIMU::Start()` 读取并固定，修改后必须重启。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `IMU_ACC_DNF_EN` | 0 | 0 关闭，1 开启 RPM 动态陷波 |
| `IMU_ACC_DNF_BW` | 15 Hz | 每个陷波器的带宽，范围 5～30 Hz |
| `IMU_ACC_DNF_HMC` | 3 | 每个电机的谐波阶数，范围 1～7 |
| `IMU_ACC_DNF_MIN` | 25 Hz | 最低基频，范围 1～1000 Hz |

这几个默认滤波参数沿用 gyro 的起始值，不代表已针对 Omni 实机整定。
禁用时不分配滤波器数组，ACC 沿用原来的积分输入。
配置非法或内存分配失败时输出 `ACC RPM notch initialization failed` 并旁路。

## 频率和异常处理

- 第 k 阶目标频率为 `abs(RPM) / 60 * k`，最多支持 8 个电机。
- 非零低转速时，使用与 gyro 类似的最低频率停驻及半带宽阶次间隔。零转速直接旁路对应电机，与 gyro 停驻行为有所区别。
- 总帧和各电机时间戳都必须有效。任何一者达到 3 秒超时、报告移除或未来时间戳均使对应滤波失效。
- 超时检查不依赖新消息，因此整个发布者停止后也能退出滤波。
- 使用连续 32 个 `sensor_accel.timestamp_sample` 间隔估计实际输入采样率。不能用 FIFO 原始 ODR 或 `IMU_INTEG_RATE` 代替。
- 采样率变化超过 1% 时更新系数；单次间隔偏离已知周期超过 25%、设备变化、丢样或时间戳回退时清除历史并重新估计采样率。重新估计期间旁路。
- 若目标中心频率加半带宽达到奈奎斯特频率，旁路该阶谐波。此位置无法消除已经由上游汇总/降采样产生的混叠。
- 非有限输入不进入递归滤波状态；原始异常值继续沿用现有输入处理。滤波计算产生非有限输出时清除滤波状态并回退原样本。

## 原始诊断保留

`sensor_accel`、ACC 均值/方差和 clipping 计数保持原始数据语义。启用时同步运行一个未陷波的 ACC 积分器，为现有 `accel_vibration_metric` 提供原始积分结果。
不应期待开启陷波后该原始振动指标下降；应同时观察 `vehicle_imu` 和 EKF 输出。没有新增日志话题，现有日志采集配置需确认包含用于对比的原始 ACC、`esc_status`、`vehicle_imu` 及估计器状态。

## 编译、启用与验证

```sh
make px4_fmu-v5_default
```

烧录固件后，可在台架上启用；没有新增模块启动项，滤波器随 `sensors` 内的 `VehicleIMU` 初始化。

```sh
param set IMU_ACC_DNF_EN 1
param save
reboot
```

重启后确认运行时参数及输入链路：

```sh
param show IMU_ACC_DNF_EN
param show IMU_ACC_DNF_BW
param show IMU_ACC_DNF_HMC
param show IMU_ACC_DNF_MIN
listener esc_status -n 5
sensors status
top
```

`sensors status` 输出各 IMU 的 `ACC RPM notch` 启用状态、实际滤波采样率，以及有效电机/谐波中心频率。
`enabled` 表示配置启用；只有有效频率行出现才表示相应陷波器当前工作。无 RPM、超时、采样率尚未建立或频率越界时可能没有有效频率。

回退：设置 `IMU_ACC_DNF_EN 0`、保存并重启。

自动测试为 [AccelRpmNotchTest.cpp](/home/jay-jie/PX4-Autopilot/src/modules/sensors/vehicle_imu/AccelRpmNotchTest.cpp)，接入 PX4 unit gtest，测试目标为 `unit-AccelRpmNotch`。覆盖旁路、频率衰减、重力和低频保留、断流超时、转速跟踪、奈奎斯特限制、异常时序、多 IMU 隔离和原始诊断积分。

实机仍需验证：转速换算与实测一致、CPU 和堆内存余量、原始/滤波后频谱、EKF 创新与速度/位置漂移。滤波改变估计器输入的幅频和相位特性；不能以波形变平滑作为飞行验证通过的依据。
