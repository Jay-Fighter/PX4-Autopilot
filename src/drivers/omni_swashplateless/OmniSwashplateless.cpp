#include "OmniSwashplateless.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdio>
#include "commander/Commander.hpp"
#include "drivers/drv_hrt.h"
#include "mathlib/math/Limits.hpp"
#include "px4_platform_common/log.h"
#include "uORB/topics/pwm_input.h"

constexpr float DEG_2_RAD = static_cast<float>(M_PI) / 180.0f;
constexpr float RAD_2_DEG = 180.0f / static_cast<float>(M_PI);

OmniSwashPlateLess::OmniSwashPlateLess() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default) {
    _single_modu_cmd_param.actuator_ctrls_ua_qgc = _param_omni_actuator_ctrls_ua.get();
    _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();
    _single_modu_cmd_param.actuator_ctrls_pha_qgc = _param_omni_actuator_ctrls_phase.get();

    //     _motor_zero_bias = _param_omni_motor_zero_flag.get();
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
#ifdef OMNI_DEBUG

    actuator_test_s test_input;
    // Read QGC slider test signal
    bool test_input_flag{false};
    if (_actuator_test_sub.update(&test_input)) {

        test_input_flag = test_input.value > 0.3f ? true : false;
        if (!test_input_flag) {
            reset_throttle_output();
        } else {
            update_test_params();

            limit_and_update_outputs();
        }

        publish_throttle();
    }

#endif

    // TODO:另一个调制指令则通过姿态环的控制输出来计算，平均升力，相位角
}

void OmniSwashPlateLess::limit_and_update_outputs() {

    // limit throttle ua
    float ua_temp = _single_modu_cmd_param.actuator_ctrls_ua_qgc * ACTUATOR_CONTROLS_TO_DSHOT;
    _single_modu_packet_cmd.throttle_ua = math::constrain(ua_temp, static_cast<float>(THROTTLE_MIN), static_cast<float>(THROTTLE_MAX));

    // limit throttle us
    _single_modu_packet_cmd.throttle_us = static_cast<float>(_single_modu_cmd_param.actuator_ctrls_us_qgc) * _single_modu_packet_cmd.throttle_ua;
    float throttle_margin = THROTTLE_MAX - _single_modu_packet_cmd.throttle_ua;

    // limit us based on the throttle margin
    if (_single_modu_packet_cmd.throttle_us > _single_modu_packet_cmd.throttle_ua) {
        _single_modu_packet_cmd.throttle_us = _single_modu_packet_cmd.throttle_ua;
    }

    if (_single_modu_packet_cmd.throttle_us > throttle_margin) {
        _single_modu_packet_cmd.throttle_us = throttle_margin;
    }

    _single_modu_packet_cmd.throttle_us = math::constrain(_single_modu_packet_cmd.throttle_us, 0.0f, static_cast<float>(THROTTLE_SIN_AMP_LIMIT));

    _single_modu_packet_cmd.index = 0;  // TODO:确定索引
    _single_modu_packet_cmd.throttle_ctrls_phase = _single_modu_cmd_param.actuator_ctrls_pha_qgc * DEG_2_RAD;
    _single_modu_packet_cmd.throttle_ctrls_lag_angle = _single_modu_cmd_param.motor_lag_angle_qgc;
    _single_modu_packet_cmd.timestamp = hrt_absolute_time();
}

void OmniSwashPlateLess::reset_throttle_output() {
    _single_modu_packet_cmd.throttle_ua = 0.0f;
    _single_modu_packet_cmd.throttle_us = 0.0f;
    _single_modu_packet_cmd.throttle_ctrls_phase = 0.0f;
    _single_modu_packet_cmd.throttle_ctrls_lag_angle = 0.0f;
    _single_modu_packet_cmd.timestamp = hrt_absolute_time();
}

void OmniSwashPlateLess::publish_throttle() {

    // publish for logging
    _single_modu_packet_cmd_pub.publish(_single_modu_packet_cmd);
}

void OmniSwashPlateLess::update_test_params() {

    // clear update
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        // update parameters from storage
        updateParams();
        _exp_mode = _param_omni_exp_mode.get();

        _single_modu_cmd_param.actuator_ctrls_ua_qgc = _param_omni_actuator_ctrls_ua.get();
        _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();
        _single_modu_cmd_param.actuator_ctrls_pha_qgc = _param_omni_actuator_ctrls_phase.get();  // deg
        _single_modu_cmd_param.motor_lag_angle_qgc = _param_motor_delay_angle_bias.get() * DEG_2_RAD;
        // _single_modu_cmd_param.motor_zero_set_flag = _param_omni_motor_zero_flag.get();  // get motor zero bias param
        // _single_modu_cmd_param.encoder_reverse_flag = _param_encoder_rev_flag.get();     // get encoder rot dir param
        _single_modu_cmd_param.timestamp = hrt_absolute_time();

        // publish
        _single_modulation_cmd_param_pub.publish(_single_modu_cmd_param);
    }

    switch (_exp_mode) {
        case 0: {
            /* Test1: Fixed us=0, ua increment with smooth ramp */
            static hrt_abstime _last_increment_time = hrt_absolute_time();
            static bool stop_increment = false;
            static float target_ua = 0.0f;  // 目标值

            float dt = 0.002f;  // Run() 循环周期 (500Hz)
            float tau = 0.1f;   // 平滑时间常数 (秒)，控制爬坡快慢
            float alpha = dt / (tau + dt);

            hrt_abstime now = hrt_absolute_time();

            if (!stop_increment && (now - _last_increment_time) > 10_s) {
                target_ua += 0.05f;
                _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                _last_increment_time = now;

                if (target_ua > 0.61f) {
                    _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.0f;
                    stop_increment = true;  // 达到上限后停止
                }
            }

            if (!stop_increment) {
                // x += alpha * (x_target - x);
                _single_modu_cmd_param.actuator_ctrls_ua_qgc += alpha * (target_ua - _single_modu_cmd_param.actuator_ctrls_ua_qgc);
            }
            break;
        }

        case 1: {
            /*Test2: Fixed ua, us increment with smooth ramp */
            static hrt_abstime _last_increment_time = hrt_absolute_time();
            static bool stop_increment = false;
            static float target_us = 0.0f;  // 目标值

            float dt = 0.002f;  // Run() 循环周期 (500Hz)
            float tau = 0.1f;   // 平滑时间常数 (秒)，控制爬坡快慢
            float alpha = dt / (tau + dt);

            hrt_abstime now = hrt_absolute_time();
            if (!stop_increment && (now - _last_increment_time) > 10_s) {
                target_us += 0.05f;
                _last_increment_time = now;

                if (target_us > 0.41f) {
                    target_us = 0.0f;
                    _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.0f;
                    _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                    stop_increment = true;  // 达到上限后停止
                }
            }
            if (!stop_increment) {
                _single_modu_cmd_param.actuator_ctrls_us_qgc += alpha * (target_us - _single_modu_cmd_param.actuator_ctrls_us_qgc);
            }
            break;
        }

        case 2: {
            /*Test3: Fixed us, ua increment with smooth ramp*/
            static hrt_abstime _last_increment_time = hrt_absolute_time();
            static bool stop_increment = false;
            static float target_ua = 0.0f;  // 目标值

            float dt = 0.002f;  // Run() 循环周期 (500Hz)
            float tau = 0.1f;   // 平滑时间常数 (秒)，控制爬坡快慢
            float alpha = dt / (tau + dt);

            hrt_abstime now = hrt_absolute_time();

            if (!stop_increment && (now - _last_increment_time) > 10_s) {

                target_ua += 0.05f;
                _last_increment_time = now;

                if (target_ua > 0.51f) {
                    target_ua = 0.0f;
                    _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.0f;
                    _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                    stop_increment = true;  // 达到上限后停止
                }
            }

            if (!stop_increment) {
                _single_modu_cmd_param.actuator_ctrls_ua_qgc += alpha * (target_ua - _single_modu_cmd_param.actuator_ctrls_ua_qgc);
            }
            break;
        }

        case 3: {
            // exp3: Fixed ua,us, phase increment 0-360°
            static hrt_abstime _last_increment_time = hrt_absolute_time();
            static bool stop_increment = false;
            static float target_phase = 0.0f;  // target phase (deg)

            float dt = 0.002f;  // Run() 循环周期 (500Hz)
            float tau = 0.1f;   // 平滑时间常数 (秒)，控制爬坡快慢
            float alpha = dt / (tau + dt);

            hrt_abstime now = hrt_absolute_time();

            if (!stop_increment && (now - _last_increment_time) > 10_s) {

                target_phase += 10.0f;  // step: 10deg
                _last_increment_time = now;

                if (target_phase > 360.0f) {
                    // target_phase = 0.0f;
                    _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
                    _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
                    stop_increment = true;  // 达到上限后停止
                }
            }

            if (!stop_increment) {
                _single_modu_cmd_param.actuator_ctrls_pha_qgc += alpha * (target_phase - _single_modu_cmd_param.actuator_ctrls_pha_qgc);
                // _single_modu_cmd_param.actuator_ctrls_pha_qgc = _single_modu_cmd_param.actuator_ctrls_pha_qgc * DEG_2_RAD;
            }
            break;
        }

        case 4: {

            /*Test4: 先给定ua，1s后再叠加上us*/
            static hrt_abstime _last_increment_time = 0;
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

            break;
        }
        default:
            break;
    }
    _single_modu_cmd_param.timestamp = hrt_absolute_time();

    // publish
    _single_modulation_cmd_param_pub.publish(_single_modu_cmd_param);
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
