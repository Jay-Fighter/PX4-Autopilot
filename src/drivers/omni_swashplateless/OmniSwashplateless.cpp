#include "OmniSwashplateless.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <cmath>
#include <cstdio>
#include "commander/Commander.hpp"
#include "drivers/drv_hrt.h"
#include "mathlib/math/Limits.hpp"
#include "px4_platform_common/log.h"

constexpr float DEG_2_RAD = static_cast<float>(M_PI) / 180.0f;
constexpr float RAD_2_DEG = 180.0f / static_cast<float>(M_PI);

OmniSwashPlateLess::OmniSwashPlateLess() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {
        _single_modu_cmd_param.actuator_ctrls_ua_qgc = _param_omni_actuator_ctrls_ua.get();
        _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();
        _single_modu_cmd_param.actuator_ctrls_pha_qgc = _param_omni_actuator_ctrls_phase.get();
        _single_modu_cmd_param.motor_lag_angle_qgc = _param_motor_delay_angle_bias.get() * DEG_2_RAD;
}

OmniSwashPlateLess::~OmniSwashPlateLess() {
        // Be inactive
        ScheduleClear();
        // Free our counters/timers
        perf_free(_loop_perf);
        perf_free(_loop_interval_perf);
}

bool OmniSwashPlateLess::init() {

        // execute Run() on every sensor_accel publication //TODO:将调度挂载为回调触发
        //     if (!_sensor_accel_sub.registerCallback()) {
        //         PX4_ERR("callback registration failed");
        //         return false;
        //     }

        // alternatively, Run on fixed interval
        ScheduleOnInterval(5_ms);  // 2000 us interval, 200 Hz rate
        return true;
}

void OmniSwashPlateLess::Run() {
        if (should_exit()) {
                ScheduleClear();
                exit_and_cleanup();
                return;
        }

        perf_begin(_loop_perf);
        perf_count(_loop_interval_perf);

        modulationCmdCal();

        perf_end(_loop_perf);
}

void OmniSwashPlateLess::modulationCmdCal() {
#ifdef OMNI_DEBUG_SINGLE

        actuator_test_s test_input;
        // Read QGC slider test signal
        static bool qgc_drag_bar_active{false};
        if (_actuator_output_sub.update(&test_input)) {

                qgc_drag_bar_active = test_input.value > 0.2f ? true : false;
        }

        if (!qgc_drag_bar_active) {
                reset_throttle_output();

                // 同时重置测试逻辑状态
                _stop_increment = false;
                _target_us = _single_modu_cmd_param.actuator_ctrls_us_qgc;
                _target_ua = _single_modu_cmd_param.actuator_ctrls_ua_qgc;
                _target_phase = _single_modu_cmd_param.actuator_ctrls_pha_qgc;
                _last_increment_time = hrt_absolute_time();

                // 检查参数更新（每次Run都可触发）
                if (_parameter_update_sub.updated()) {
                        parameter_update_s param_update;
                        _parameter_update_sub.copy(&param_update);

                        updateParams();

                        // 立即同步参数到命令结构
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = _param_omni_actuator_ctrls_ua.get();
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();
                        _single_modu_cmd_param.actuator_ctrls_pha_qgc = _param_omni_actuator_ctrls_phase.get();
                        _single_modu_cmd_param.motor_lag_angle_qgc = _param_motor_delay_angle_bias.get() * DEG_2_RAD;
                } else {
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = _param_omni_actuator_ctrls_ua.get();
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();
                        _single_modu_cmd_param.actuator_ctrls_pha_qgc = _param_omni_actuator_ctrls_phase.get();  // deg
                        _single_modu_cmd_param.motor_lag_angle_qgc = _param_motor_delay_angle_bias.get() * DEG_2_RAD;
                        _single_modu_cmd_param.timestamp = hrt_absolute_time();
                }

        } else {
                update_test_params();
                limit_and_update_outputs();
        }

        publish_throttle();

#endif

#ifndef OMNI_DEBUG_SINGLE
        // TODO:另一个调制指令则通过姿态环的控制输出来计算，平均升力，相位角

        // add by jayjie
        if (_parameter_update_sub.updated()) {
                parameter_update_s param_update;
                _parameter_update_sub.copy(&param_update);

                updateParams();
                _single_modu_cmd_param.motor_lag_angle_qgc = _param_motor_delay_angle_bias.get() * DEG_2_RAD;
        }
        // end

// add by jayjie
#if OMNI_RC_MANUAL_SIM
        manual_control_setpoint_s manual_control_setpoint{};

        if (_manual_control_setpoint_sub.copy(&manual_control_setpoint) && manual_control_setpoint.valid &&
            hrt_elapsed_time(&manual_control_setpoint.timestamp) < 500_ms) {
                limit_and_update_outputs(manual_control_setpoint);

        } else {
                for (size_t i = 0; i < OMNI_ACTUATOR_NUM; i++) {
                        _omni_outputs_cmd_groups.throttle_ua[i] = 0.0f;
                        _omni_outputs_cmd_groups.throttle_us[i] = 0.0f;
                        _omni_outputs_cmd_groups.throttle_ctrls_flap[i] = 0.0f;
                        _omni_outputs_cmd_groups.throttle_ctrls_phase[i] = 0.0f;
                        _omni_outputs_cmd_groups.throttle_ctrls_lag_angle[i] = 0.0f;
                        _omni_outputs_cmd_groups.index[i] = i;
                }

                _omni_outputs_cmd_groups.timestamp = hrt_absolute_time();
                _omni_outputs_cmd_groups_pub.publish(_omni_outputs_cmd_groups);
        }
#else
        // end
        omni_actuator_setpoint_s omni_actuator_setpoint;

        if (_omni_actuator_setpoint_sub.update(&omni_actuator_setpoint)) {
                limit_and_update_outputs(omni_actuator_setpoint);
        }
// add by jayjie
#endif
// end
#endif
}

void OmniSwashPlateLess::limit_and_update_outputs(omni_actuator_setpoint_s& output) {

        for (size_t i = 0; i < output.num_groups; i++) {
                float ua = std::fabs(output.fz[i]) * ACTUATOR_CONTROLS_TO_DSHOT;
                float us = sqrtf((output.fx[i] * output.fx[i] + output.fy[i] * output.fy[i]) / 2.0f) * ACTUATOR_CONTROLS_TO_DSHOT;
                // ua = 0.15*ACTUATOR_CONTROLS_TO_DSHOT;
                // us = 0;
                float phase = atan2f(output.fy[i], output.fx[i]);
                if (phase < 0.0f) {
                        phase += 2.0f * static_cast<float>(M_PI);
                }

                _omni_outputs_cmd_groups.throttle_ua[i] = math::constrain(ua, static_cast<float>(THROTTLE_MIN), static_cast<float>(THROTTLE_MAX));

                _omni_outputs_cmd_groups.throttle_us[i] = math::constrain(us, 0.0f, us_2_ua_ratio_max * _omni_outputs_cmd_groups.throttle_ua[i]);

                _omni_outputs_cmd_groups.throttle_ctrls_flap[i] = acosf(std::fabs(output.fz[i]) / output.f[i]);

                _omni_outputs_cmd_groups.throttle_ctrls_phase[i] = phase;

                _omni_outputs_cmd_groups.throttle_ctrls_lag_angle[i] = _single_modu_cmd_param.motor_lag_angle_qgc;

                _omni_outputs_cmd_groups.index[i] = i;
        }
        _omni_outputs_cmd_groups.timestamp = hrt_absolute_time();

        _omni_outputs_cmd_groups_pub.publish(_omni_outputs_cmd_groups);
}

// add by jayjie
float OmniSwashPlateLess::getManualTestMotorPhase(float phase_body, size_t motor_index) const {
        const float two_pi = 2.0f * static_cast<float>(M_PI);
        static constexpr float inv_sqrt2 = 0.70710678118f;
        static constexpr float xh_axis_body[OMNI_ACTUATOR_NUM][2] = {
                { inv_sqrt2, inv_sqrt2 },
                { -inv_sqrt2, -inv_sqrt2 },
                { inv_sqrt2, -inv_sqrt2 },
                { -inv_sqrt2, inv_sqrt2 }
        };
        static constexpr float yh_axis_body[OMNI_ACTUATOR_NUM][2] = {
                { -inv_sqrt2, inv_sqrt2 },
                { inv_sqrt2, -inv_sqrt2 },
                { inv_sqrt2, inv_sqrt2 },
                { -inv_sqrt2, -inv_sqrt2 }
        };

        const float x_body = cosf(phase_body);
        const float y_body = sinf(phase_body);

        const float x_local = x_body * xh_axis_body[motor_index][0] + y_body * xh_axis_body[motor_index][1];
        const float y_local = x_body * yh_axis_body[motor_index][0] + y_body * yh_axis_body[motor_index][1];

        float motor_phase = atan2f(y_local, x_local);

        if (motor_phase < 0.0f) {
                motor_phase += two_pi;
        }

        return motor_phase;
}

void OmniSwashPlateLess::limit_and_update_outputs(const manual_control_setpoint_s& manual_control_setpoint) {

        const float throttle_norm = math::constrain((1.0f - manual_control_setpoint.throttle) * 0.5f, 0.0f, 1.0f);
        const float roll = math::constrain(manual_control_setpoint.roll, -1.0f, 1.0f);
        const float roll_abs = math::constrain(std::fabs(roll), 0.0f, 1.0f);

        const float ua =
            math::constrain(throttle_norm * ACTUATOR_CONTROLS_TO_DSHOT, static_cast<float>(THROTTLE_MIN), static_cast<float>(THROTTLE_MAX));

        const float us = math::constrain(roll_abs * us_2_ua_ratio_max * ua, 0.0f, us_2_ua_ratio_max * ua);

        const float yaw = math::constrain(manual_control_setpoint.yaw, -1.0f, 1.0f);
        const float pitch = math::constrain(manual_control_setpoint.pitch, -1.0f, 1.0f);
        const float phase_stick_norm = sqrtf(yaw * yaw + pitch * pitch);

        float phase = 0.0f;

        if (phase_stick_norm > 0.05f) {
                phase = atan2f(yaw, pitch);

                if (phase < 0.0f) {
                        phase += 2.0f * static_cast<float>(M_PI);
                }
        }

        const bool test_all_outputs = (_param_omni_rc_test_mode.get() == 1);
        const size_t active_index = static_cast<size_t>(math::constrain(_param_omni_rc_test_index.get(), int32_t{0}, int32_t{OMNI_ACTUATOR_NUM - 1}));

        for (size_t i = 0; i < OMNI_ACTUATOR_NUM; i++) {
                const bool active_output = test_all_outputs || (i == active_index);
                const float motor_phase = getManualTestMotorPhase(phase, i);

                // _omni_outputs_cmd_groups.throttle_ua[i] = active_output ? ua : 0.0f;
		_omni_outputs_cmd_groups.throttle_ua[i] = phase;
                _omni_outputs_cmd_groups.throttle_us[i] = active_output ? us : 0.0f;
                _omni_outputs_cmd_groups.throttle_ctrls_flap[i] = 0.0f;
                _omni_outputs_cmd_groups.throttle_ctrls_phase[i] = active_output ? motor_phase : 0.0f;
                _omni_outputs_cmd_groups.throttle_ctrls_lag_angle[i] = active_output ? _single_modu_cmd_param.motor_lag_angle_qgc : 0.0f;
                _omni_outputs_cmd_groups.index[i] = i;
        }

        _omni_outputs_cmd_groups.timestamp = hrt_absolute_time();
        _omni_outputs_cmd_groups_pub.publish(_omni_outputs_cmd_groups);
}
// end

void OmniSwashPlateLess::limit_and_update_outputs() {

        // limit throttle ua
        float ua_temp = _single_modu_cmd_param.actuator_ctrls_ua_qgc * ACTUATOR_CONTROLS_TO_DSHOT;
        _single_output_cmd.throttle_ua = math::constrain(ua_temp, static_cast<float>(THROTTLE_MIN), static_cast<float>(THROTTLE_MAX));

        // limit throttle us
        _single_output_cmd.throttle_us = static_cast<float>(_single_modu_cmd_param.actuator_ctrls_us_qgc) * _single_output_cmd.throttle_ua;
        float throttle_margin = THROTTLE_MAX - _single_output_cmd.throttle_ua;

        // limit us based on the throttle margin
        if (_single_output_cmd.throttle_us > _single_output_cmd.throttle_ua) {
                _single_output_cmd.throttle_us = _single_output_cmd.throttle_ua;
        }

        if (_single_output_cmd.throttle_us > throttle_margin) {
                _single_output_cmd.throttle_us = throttle_margin;
        }

        _single_output_cmd.throttle_us = math::constrain(_single_output_cmd.throttle_us, 0.0f, static_cast<float>(THROTTLE_SIN_AMP_LIMIT));

        _single_output_cmd.index = 0;  // TODO:确定索引
        _single_output_cmd.throttle_ctrls_phase = _single_modu_cmd_param.actuator_ctrls_pha_qgc * DEG_2_RAD;
        _single_output_cmd.throttle_ctrls_lag_angle = _single_modu_cmd_param.motor_lag_angle_qgc;
        _single_output_cmd.timestamp = hrt_absolute_time();
}

void OmniSwashPlateLess::reset_throttle_output() {
        _single_output_cmd.throttle_ua = 0.0f;
        _single_output_cmd.throttle_us = 0.0f;
        _single_output_cmd.throttle_ctrls_phase = 0.0f;
        _single_output_cmd.throttle_ctrls_lag_angle = 0.0f;
        _single_output_cmd.timestamp = hrt_absolute_time();
}

void OmniSwashPlateLess::publish_throttle() {

        // publish for logging
        _single_output_cmd_pub.publish(_single_output_cmd);
}

void OmniSwashPlateLess::update_test_params() {

        // clear update
        if (_parameter_update_sub.updated()) {
                parameter_update_s param_update;
                _parameter_update_sub.copy(&param_update);

                // update parameters from storage
                updateParams();

                _single_modu_cmd_param.actuator_ctrls_ua_qgc = _param_omni_actuator_ctrls_ua.get();
                _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();
                _single_modu_cmd_param.actuator_ctrls_pha_qgc = _param_omni_actuator_ctrls_phase.get();  // deg
                _single_modu_cmd_param.motor_lag_angle_qgc = _param_motor_delay_angle_bias.get() * DEG_2_RAD;
                _single_modu_cmd_param.timestamp = hrt_absolute_time();

                // publish
                // _single_output_cmd_param_pub.publish(_single_modu_cmd_param);
        }

#if OMNI_TEST_MODE_SELECTED == 0
        /* Test1: Fixed us=0, ua increment with smooth ramp
         * ua(t) = ua_base + (t / UA_RAMP_TIME) * UA_MAX
         * us(t) = 0
         */

        static float ua_base = _param_omni_actuator_ctrls_ua.get();
        static float ua_max = 0.5f;
        static float ua_ramp_time = 30.0f;  // 30s arrive at max value

        hrt_abstime now = hrt_absolute_time();

        float t = (now - _last_increment_time) * 1e-6f;

        if (!_stop_increment) {
                _target_ua = ua_base + ua_max * (t / ua_ramp_time);
                _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;  // 固定 us=0

                if (_target_ua > 0.51f) {
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                        _stop_increment = true;  // 达到上限后停止
                }
        }

        if (!_stop_increment) {
                _single_modu_cmd_param.actuator_ctrls_ua_qgc = _target_ua;
        }

#elif OMNI_TEST_MODE_SELECTED == 1
        /* Test2: Fixed ua, us increment with smooth ramp
         * us(t) = us_base + (t / US_RAMP_TIME) * US_MAX
         * ua(t) = constant
         */

        static float us_base = _param_omni_actuator_ctrls_us.get();

        static float us_max = 0.30f;

        static float us_ramp_time = 15.0f;  // 15s arrive at max value

        static float hold_time = 2.0f;

        hrt_abstime now = hrt_absolute_time();
        float t = (now - _last_increment_time) * 1e-6f;
        if (t < hold_time) {

                _single_modu_cmd_param.actuator_ctrls_ua_qgc = _single_modu_cmd_param.actuator_ctrls_ua_qgc;

        } else {
                // ====== 归一化进度（确保同步） ======
                float s = (t - hold_time) / us_ramp_time;

                if (!_stop_increment) {
                        _target_us = us_base + (us_max - us_base) * s;

                        if (_target_us > 0.31f) {
                                _target_us = 0.0f;
                                _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                                _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                                _stop_increment = true;  // 达到上限后停止
                        }
                }

                if (!_stop_increment) {
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = _target_us;
                }
        }

#elif OMNI_TEST_MODE_SELECTED == 2
        /* Test3: Fixed us, ua increment with smooth ramp
         * ua(t) = ua_base + (t / UA_RAMP_TIME) * UA_MAX
         * us(t) = constant
         */

        static float ua_base = _param_omni_actuator_ctrls_ua.get();

        static float ua_max = 0.5f;

        static float ua_ramp_time = 20.0f;  // 15s arrive at max value

        hrt_abstime now = hrt_absolute_time();

        float t = (now - _last_increment_time) * 1e-6f;

        if (!_stop_increment) {

                _target_ua = ua_base + ua_max * (t / ua_ramp_time);  // 10 秒内线性从 0→0.5

                if (_target_ua > 0.51f) {
                        _target_ua = 0.0f;
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                        _stop_increment = true;  // 达到上限后停止
                }
        }

        if (!_stop_increment) {
                _single_modu_cmd_param.actuator_ctrls_ua_qgc = _target_ua;
        }

#elif OMNI_TEST_MODE_SELECTED == 3

        /**
         * Test4: phase increases monotonically (0 → 2π → 4π → 6π → ...)
         * with smooth sinusoidal motion inside each cycle.
         */

        static float time_period = 10.0f;  // 每 10 秒一圈
        static int cycles_target = 5;      // 转 5 圈
        static float hold_time = 2.0f;     // ★ 前置保持 3 秒

        hrt_abstime now = hrt_absolute_time();
        float t = (now - _last_increment_time) * 1e-6f;

        if (t < hold_time) {

        } else {

                if (!_stop_increment) {
                        float T = t - hold_time;

                        // ==== 总时间内已完成多少周期 ====
                        float cycles_f = T / time_period;
                        int cycles_int = (int)floorf(cycles_f);  // 已完成整圈数

                        // ==== 当前周期位置 0~T ====
                        float phase_t = fmodf(T, time_period);

                        // ==== 归一化 s: 0~1 ====
                        float s = phase_t / time_period;

                        // ==== 单圈平滑相位 0→2π（余弦半周期）====
                        float local_phase = static_cast<float>(M_PI) * (1.0f - cosf(static_cast<float>(M_PI) * s));  // 0→2π

                        // ==== 累积相位 0→2π→4π→6π ====
                        float phase_rad = cycles_int * 2.0f * static_cast<float>(M_PI) + local_phase;

                        // ==== 输出仍然映射到 0~360° ====
                        float phase_deg_mod = fmodf(phase_rad * 180.0f / static_cast<float>(M_PI), 360.0f);

                        _single_modu_cmd_param.actuator_ctrls_pha_qgc = phase_deg_mod;

                        // ==== 完成 cycles_target 圈 ====
                        if (phase_rad >= cycles_target * 2.0f * static_cast<float>(M_PI)) {
                                _stop_increment = true;
                        }

                } else {
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                }
        }
#elif OMNI_TEST_MODE_SELECTED == 4
        /*Test5: 先给定ua，1s后再叠加上us*/
        static float last_ua_param = NAN;

        hrt_abstime now = hrt_absolute_time();

        // 检测参数更新
        float ua_now = _param_omni_actuator_ctrls_ua.get();

        // 当 ua 被修改时，重新启动延迟
        if (!PX4_ISFINITE(last_ua_param) || fabsf(ua_now - last_ua_param) > 1e-5f) {
                _last_increment_time = now;
        }

        // 更新记录
        last_ua_param = ua_now;

        // === 延迟逻辑 ===
        if ((now - _last_increment_time) < 1_s) {
                _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;  // 前 1 s 不加 us
        } else {
                _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();  // 之后恢复 us
        }

#elif OMNI_TEST_MODE_SELECTED == 5
        /* Test6: Fixed ua, phase, triangle wave on us (0 → us_max → 0) until count > cycles_target
         * us(t) = (t / UA_RAMP_TIME) * 2PI
         * ua(t), phase(t) = constant
         */
        float us_max = 0.26f;
        float T = 6.0f;                // 一个周期（秒）
        static int cycle_count = 0;    // 已完成周期数
        static bool finished = false;  // 已完成 6 周停止

        hrt_abstime now = hrt_absolute_time();
        float t = (now - _last_increment_time) * 1e-6f;  // elapsed [s]

        if (finished) {
                _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;  // 固定UA或你想要的值
                return;
        }

        // ======= 当前相位 =======
        float current_time = fmodf(t, T);

        // ======= 检查周期是否完成 =======
        static float last_phase = 0.0f;
        if (current_time < last_phase) {
                cycle_count++;
                if (cycle_count >= 6) {  // 只执行 6 次三角波
                        finished = true;
                }
        }
        last_phase = current_time;

        // ======= 三角波计算 =======
        float ramp;
        if (current_time < T * 0.5f) {
                ramp = current_time / (T * 0.5f);  // 0 → 1
        } else {
                ramp = 2.0f - (current_time / (T * 0.5f));  // 1 → 0
        }

        // ======= 映射输出 =======
        _single_modu_cmd_param.actuator_ctrls_us_qgc = ramp * us_max;

#elif OMNI_TEST_MODE_SELECTED == 6
        /* Test4: Fixed ua, us, phase sweeps 0 → 2π → 0 smoothly (cosine-based)
         * phase(t) = pi * (1 - cos(2*pi * t / T))
         */

        static int cycles_target = 5;      // 扫描 5 次
        static float time_period = 10.0f;  // 每次 10s

        hrt_abstime now = hrt_absolute_time();
        float t = (now - _last_increment_time) * 1e-6f;

        if (!_stop_increment) {

                float phase_t = fmodf(t, time_period);  // 一个周期内时间
                float count = t / time_period;          // 总执行了多少周期

                // ======== 平滑相位函数：0 → 2π → 0 ========
                float phase_rad = static_cast<float>(M_PI) * (1.0f - cosf(2.0f * static_cast<float>(M_PI) * (phase_t / time_period)));

                // 转为度数输出
                float phase_deg = phase_rad * 180.0f / static_cast<float>(M_PI);

                _single_modu_cmd_param.actuator_ctrls_pha_qgc = phase_deg;

                // ======== 结束判定 ========
                if (count >= cycles_target) {
                        _stop_increment = true;
                }

        } else {
                _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
        }

#elif OMNI_TEST_MODE_SELECTED == 7
        /**
         * Test8: Fixed phase, ua(t) & us(t) increase simultaneously
         * ua: 0 → 0.6
         * us: 0 → 0.3
         * phase: constant (e.g., 45 deg)
         */

        static float ua_max = 0.55f;
        static float us_max = 0.3f;
        static float ua_base = _param_omni_actuator_ctrls_ua.get();
        static float us_base = _param_omni_actuator_ctrls_us.get();
        static float ramp_time = 10.0f;  // 两者都在 20 秒内到达最大值
        static float hold_time = 3.0f;   // ★ 前置保持 3 秒

        hrt_abstime now = hrt_absolute_time();
        float t = (now - _last_increment_time) * 1e-6f;
        if (t < hold_time) {

                _single_modu_cmd_param.actuator_ctrls_ua_qgc = ua_base;

        } else {
                // ====== 归一化进度（确保同步） ======
                float s = (t - hold_time) / ramp_time;
                if (s > 1.0f)
                        _stop_increment = true;

                if (!_stop_increment) {

                        // ====== 线性同步递增 ======
                        float ua = ua_base + (ua_max - ua_base) * s;
                        float us = us_base + (us_max - us_base) * s;

                        // 限幅
                        if (ua > ua_max)
                                ua = ua_max;
                        if (us > us_max)
                                us = us_max;

                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = ua;
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = us;

                } else {
                        // ====== 停止后 reset ======
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                }
        }

#elif OMNI_TEST_MODE_SELECTED == 8
        /**
         * Test9: Fixed us, ua(t) & phase(t) increase simultaneously
         * ua: ua_base → ua_max
         * phase: 0° → 360°
         * us: constant (us_base)
         */

        static float ua_base = _param_omni_actuator_ctrls_ua.get();

        static float ua_max = 0.55f;
        static float ramp_time = 20.0f;  // ua 和 phase 在 20 秒内同时到达最大值

        static float hold_time = 3.0f;  // ★ 前置保持 3 秒

        hrt_abstime now = hrt_absolute_time();
        float t = (now - _last_increment_time) * 1e-6f;

        if (t < hold_time) {

                _single_modu_cmd_param.actuator_ctrls_ua_qgc = ua_base;

        } else {

                // ====== 归一化进度（确保同步） ======
                float s = (t - hold_time) / ramp_time;
                if (s > 1.0f)
                        _stop_increment = true;

                if (!_stop_increment) {

                        // ====== ua 同步递增 ======
                        float ua = ua_base + (ua_max - ua_base) * s;

                        // ====== phase 同步递增 ======
                        float phase_deg = static_cast<float>(M_PI) * (1.0f - cosf(static_cast<float>(M_PI) * s)) * RAD_2_DEG;  // to degree

                        // ====== 输出 ======
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = ua;
                        _single_modu_cmd_param.actuator_ctrls_pha_qgc = phase_deg;

                } else {
                        // ====== reset ======
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                }
        }

#elif OMNI_TEST_MODE_SELECTED == 9
        /**
         * Test10: Fixed ua, us(t) & phase(t) increase simultaneously
         * us: 0 → us_max
         * phase: 0° → 360° (one full rotation)
         * ua: constant
         */

        static float us_max = 0.3f;
        static float ramp_time = 20.0f;  // us 和 phase 同步在 20s 内完成

        static float hold_time = 3.0f;  // ★ 前置保持 3 秒

        hrt_abstime now = hrt_absolute_time();
        float t = (now - _last_increment_time) * 1e-6f;

        if (t < hold_time) {

                // hold
        } else {

                // ====== 归一化进度（确保同步） ======
                float s = (t - hold_time) / ramp_time;
                if (s > 1.0f)
                        _stop_increment = true;

                if (!_stop_increment) {

                        // ====== us 线性递增 ======
                        float us = us_max * s;

                        // ====== phase 同步递增 ======
                        float phase_deg = static_cast<float>(M_PI) * (1.0f - cosf(static_cast<float>(M_PI) * s)) * RAD_2_DEG;  // to degree

                        // ====== 输出 ======
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = us;
                        _single_modu_cmd_param.actuator_ctrls_pha_qgc = phase_deg;

                } else {
                        // ====== reset ======
                        _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                        _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                }
        }
#elif OMNI_TEST_MODE_SELECTED == 10
        /**
         * Test11: Dynamic us, ua(t) & phase(t) increase simultaneously
         */

        // ===== 你给定的序列（示例，替换成你的数组）=====
        static constexpr int kN = 8;
        static constexpr float kUaSeq[kN] = {0.45f, 0.45f, 0.4f, 0.45f, 0.5f, 0.45f, 0.45f, 0.05f};
        static constexpr float kUsSeq[kN] = {0.0, 0.25, 0.10, 0.25, 0.10, 0.25, 0.25, 0.0};
        static constexpr float kPhaseSeq[kN] = {0.f, 45.f, 90.f, 135.f, 180.f, 225.f, 45.f, 0.0f};

        // ===== 时序参数 =====
        static constexpr hrt_abstime kGroupDuration = 4_s;  // 每组 2s
        static constexpr hrt_abstime kUsDelay = 1_s;        // 仅第 1 组：us 延迟 1s

        // ===== 运行状态 =====
        static bool _seq_started = false;
        static int _seq_idx = 0;
        static hrt_abstime _group_start_time = 0;

        const hrt_abstime now = hrt_absolute_time();

        // 1) 首次启动：进入第 0 组
        if (!_seq_started) {
                _seq_started = true;
                _seq_idx = 0;
                _group_start_time = now;
        }

        // 2) 组切换：每 2s 前进一组
        while ((now - _group_start_time) >= kGroupDuration) {
                _group_start_time += kGroupDuration;

                if (_seq_idx < (kN - 1)) {
                        _seq_idx++;
                } else {
                        // 到最后一组后不再推进
                        break;
                }
        }

        // 3) 当前组的目标值
        const float ua_cmd = kUaSeq[_seq_idx];
        const float us_cmd = kUsSeq[_seq_idx];
        const float phase_cmd = kPhaseSeq[_seq_idx];

        // 4) 输出：第 0 组 us 延迟 1s；其余组同步切换
        const hrt_abstime t_in_group = now - _group_start_time;

        float us_out = us_cmd;
        if ((_seq_idx == 0) && (t_in_group < kUsDelay)) {
                us_out = 0.0f;
        }

        // 5) 下发
        _single_modu_cmd_param.actuator_ctrls_ua_qgc = ua_cmd;
        _single_modu_cmd_param.actuator_ctrls_us_qgc = us_out;
        _single_modu_cmd_param.actuator_ctrls_pha_qgc = phase_cmd;

#endif

        _single_modu_cmd_param.timestamp = hrt_absolute_time();

        // publish
        _single_output_cmd_param_pub.publish(_single_modu_cmd_param);
}

int OmniSwashPlateLess::task_spawn(int argc, char* argv[]) {
        OmniSwashPlateLess* instance = new OmniSwashPlateLess();

        if (instance) {
                _object.store(instance);
                _task_id = task_id_is_work_queue;

                if (instance->init()) {
                        return PX4_OK;
                }

        } else {
                PX4_ERR("alloc failed");
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;

        return PX4_ERROR;
}

int OmniSwashPlateLess::custom_command(int argc, char* argv[]) {
        return print_usage("unknown command");
}

int OmniSwashPlateLess::print_usage(const char* reason) {
        if (reason) {
                PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Example of a omniswashplateless module running out of a work queue.

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("omni_swashplateless", "drivers/dshot");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

int OmniSwashPlateLess::print_status() {
        perf_print_counter(_loop_perf);
        perf_print_counter(_loop_interval_perf);
        return 0;
}

extern "C" __EXPORT int omni_swashplateless_main(int argc, char* argv[]) {
        return OmniSwashPlateLess::main(argc, argv);
}
