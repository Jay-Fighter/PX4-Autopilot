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
#ifdef OMNI_DEBUG

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

    // TODO:另一个调制指令则通过姿态环的控制输出来计算，平均升力，相位角
}

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
    /* Test1: Fixed us=0, ua increment with smooth ramp */

    hrt_abstime now = hrt_absolute_time();

    if (!_stop_increment && (now - _last_increment_time) > 10_s) {
        _target_ua += 0.05f;
        _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;  // 固定 us=0
        _last_increment_time = now;

        if (_target_ua > 0.56f) {
            _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.0f;
            _stop_increment = true;  // 达到上限后停止
        }
    }

    if (!_stop_increment) {
        // x += alpha * (x_target - x);
        _single_modu_cmd_param.actuator_ctrls_ua_qgc += (_target_ua - _single_modu_cmd_param.actuator_ctrls_ua_qgc);
    }

#elif OMNI_TEST_MODE_SELECTED == 1
    /*Test2: Fixed ua, us increment with smooth ramp */

    hrt_abstime now = hrt_absolute_time();
    if (!_stop_increment && (now - _last_increment_time) > 10_s) {
        _target_us += 0.05f;
        _last_increment_time = now;

        if (_target_us > 0.31f) {
            _target_us = 0.0f;
            _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.0f;
            _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
            _stop_increment = true;  // 达到上限后停止
        }
    }
    if (!_stop_increment) {
        _single_modu_cmd_param.actuator_ctrls_us_qgc += (_target_us - _single_modu_cmd_param.actuator_ctrls_us_qgc);
    }

#elif OMNI_TEST_MODE_SELECTED == 2
    /*Test3: Fixed us, ua increment with smooth ramp*/

    hrt_abstime now = hrt_absolute_time();

    if (!_stop_increment && (now - _last_increment_time) > 10_s) {

        _target_ua += 0.05f;
        _last_increment_time = now;

        if (_target_ua > 0.51f) {
            _target_ua = 0.0f;
            _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.0f;
            _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
            _stop_increment = true;  // 达到上限后停止
        }
    }

    if (!_stop_increment) {
        _single_modu_cmd_param.actuator_ctrls_ua_qgc += (_target_ua - _single_modu_cmd_param.actuator_ctrls_ua_qgc);
    }

#elif OMNI_TEST_MODE_SELECTED == 3

    // exp3: Fixed ua,us, phase increment 0-360°

    hrt_abstime now = hrt_absolute_time();

    if (!_stop_increment && (now - _last_increment_time) > 10_s) {

        _target_phase += 10.0f;  // step: 10deg
        _last_increment_time = now;

        if (_target_phase > 360.0f) {
            // target_phase = 0.0f;
            _single_modu_cmd_param.actuator_ctrls_ua_qgc = 0.05f;
            _single_modu_cmd_param.actuator_ctrls_us_qgc = 0.0f;
            _stop_increment = true;  // 达到上限后停止
        }
    }

    if (!_stop_increment) {
        _single_modu_cmd_param.actuator_ctrls_pha_qgc += (_target_phase - _single_modu_cmd_param.actuator_ctrls_pha_qgc);
    }

#elif OMNI_TEST_MODE_SELECTED == 4
    /*Test4: 先给定ua，1s后再叠加上us*/
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
