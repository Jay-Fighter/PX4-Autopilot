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

OmniSwashPlateLess::OmniSwashPlateLess() : ModuleParams(nullptr) {
    _single_modu_cmd_param.actuator_ctrls_ua_qgc = _param_omni_actuator_ua.get();
    _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();
    _single_modu_cmd_param.actuator_ctrls_pha_qgc = _param_omni_actuator_ctrls_phase.get();

    _pwm_input_cap.max_pulse_width = SENSOR_PWM_MAX;
    _pwm_input_cap.min_pulse_width = SENSOR_PWM_MIN;
    _motor_zero_bias = _param_omni_motor_zero_bias.get() * DEG_2_RAD;
}

void OmniSwashPlateLess::mortorStateEstimate() {
    // copy data
    _pwm_input_cap.last_pulse_width = _pwm_input_cap.pulse_width;
    _pwm_input_cap.pulse_width = pwm_input_cap_data.pulse_width;
    _pwm_input_cap.period = pwm_input_cap_data.period;
    _pwm_input_cap.timestamp = pwm_input_cap_data.timestamp;
    _pwm_input_cap.dt = static_cast<float>(_pwm_input_cap.timestamp - _pwm_input_cap.last_timestamp) / 1000000.0f;  // us to s
    _pwm_input_cap.last_timestamp = _pwm_input_cap.timestamp;

    // cal motor angle and velocity based the PWM captured
    _motor_telemetry.obs_angle_rad_last = _motor_telemetry.obs_angle_rad;
    _motor_telemetry.obs_angle_deg_last = _motor_telemetry.obs_angle_deg;

    _motor_telemetry.obs_angle_rad = motorAngleCal(_pwm_input_cap);
    _motor_telemetry.obs_angle_deg = _motor_telemetry.obs_angle_rad * RAD_2_DEG;

    _motor_telemetry.obs_rpm = motorVelocityCal(_pwm_input_cap);

    _motor_telemetry.timestamp = hrt_absolute_time();

    _motor_telemetry_pub.publish(_motor_telemetry);
    _single_pwm_input_cap_pub.publish(_pwm_input_cap);

    _single_modu_cmd.pulse_angle_rad = _motor_telemetry.obs_angle_rad;
}

float OmniSwashPlateLess::motorAngleCal(omni_pwm_cap_s& pwm_input_cap) {

    // calculate the pulse range
    pwm_input_cap.pulse_width_range = abs(pwm_input_cap.max_pulse_width - pwm_input_cap.min_pulse_width);

    // calculate the motor angle
    float rotor_angle = float(pwm_input_cap.pulse_width - pwm_input_cap.min_pulse_width) / float(pwm_input_cap.pulse_width_range) * 360.0f;

    // add the angle limit
    if (rotor_angle > 360) {
        rotor_angle = 360.0f;
    } else if (rotor_angle < 0) {
        rotor_angle = 0.0f;
    }

    // add the angle bias
    rotor_angle -= _motor_zero_bias * RAD_2_DEG;

    // remap to 0~360 deg
    if (rotor_angle < 0)
        rotor_angle += 360;

    return rotor_angle * DEG_2_RAD;
}

float OmniSwashPlateLess::motorVelocityCal(omni_pwm_cap_s& pwm_input_cap) {
    float motor_velocity = 0.0f;
    float delta_angle = _motor_telemetry.obs_angle_rad - _motor_telemetry.obs_angle_rad_last;

    if (delta_angle > M_PI_F) {
        delta_angle -= 2.0f * M_PI_F;  // 例如 1° → 359°
    } else if (delta_angle < -M_PI_F) {
        delta_angle += 2.0f * M_PI_F;  // 例如 359° → 1°
    }

    if (pwm_input_cap.dt > 1e-6f) {
        motor_velocity = delta_angle / pwm_input_cap.dt;
    }

    float rpm_instant = motor_velocity * 60.0f / (2.0f * M_PI_F);  // to rpm

    static float rpm_filtered = 0.0f;
    const float alpha = 0.2f;  // Filter coefficient
    rpm_filtered = rpm_filtered + alpha * (rpm_instant - rpm_filtered);

    return rpm_filtered;
}

void OmniSwashPlateLess::modulationCmdCal() {
#ifdef OMNI_DEBUG
    update_test_params();

    float ua_temp = _single_modu_cmd_param.actuator_ctrls_ua_qgc * ACTUATOR_CONTROLS_TO_DSHOT;
    ua_temp = math::constrain(ua_temp, static_cast<float>(THROTTLE_MIN), static_cast<float>(THROTTLE_MAX));
    _single_modu_cmd.actuator_ctrls_ua = ua_temp;

    // for debug: The amplitude of the sine is a percentage of the throttle
    _single_modu_cmd.actuator_ctrls_us = static_cast<float>(_single_modu_cmd_param.actuator_ctrls_us_qgc) * ua_temp;

    _single_modu_cmd.actuator_ctrls_pha = _single_modu_cmd_param.actuator_ctrls_pha_qgc;

    mix_throttle();
#endif
}

void OmniSwashPlateLess::mix_throttle() {
    float throttle_margin = THROTTLE_MAX - _single_modu_cmd.actuator_ctrls_ua;

    // limnit the sin amp based on the throttle margin
    if (_single_modu_cmd.actuator_ctrls_us > _single_modu_cmd.actuator_ctrls_ua) {
        _single_modu_cmd.actuator_ctrls_us = _single_modu_cmd.actuator_ctrls_ua;
    }
    if (_single_modu_cmd.actuator_ctrls_us > throttle_margin) {
        _single_modu_cmd.actuator_ctrls_us = throttle_margin;
    }

    _single_modu_cmd.actuator_ctrls_us = math::constrain(_single_modu_cmd.actuator_ctrls_us, 0.0f, static_cast<float>(THROTTLE_SIN_AMP_LIMIT));

    // calculate the throttle value for dshot
    float throttle_dc = _single_modu_cmd.actuator_ctrls_ua;  // DC component of the throttle

    float throttle_sin = _single_modu_cmd.actuator_ctrls_us * cosf(_single_modu_cmd.pulse_angle_rad - _single_modu_cmd.actuator_ctrls_pha -
                                                                   static_cast<float>(MOTOR_DELAY_ANLGE_BIAS));  // Sine component of the throttle

    _single_modu_cmd.throttle = static_cast<uint16_t>(throttle_dc + throttle_sin);
    _single_modu_cmd.throttle = constrain(_single_modu_cmd.throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    _single_modu_cmd.throttle_dc = throttle_dc;
    _single_modu_cmd.throttle_ac = throttle_sin;
    _single_modu_cmd.timestamp = hrt_absolute_time();

    // publish for logging
    _single_modulation_cmd_pub.publish(_single_modu_cmd);
}

uint16_t OmniSwashPlateLess::speedCtrl4Dshot(bool on_flag) {

    if (pwm_input_sub.update(&pwm_input_cap_data)) {
        // motor state estimate based on PWM input
        mortorStateEstimate();

        modulationCmdCal();
    }

    uint16_t throttle_2_dshot = _single_modu_cmd.throttle;
    return throttle_2_dshot;
}

void OmniSwashPlateLess::update_test_params() {

    // clear update

    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        // update parameters from storage
        updateParams();
        _single_modu_cmd_param.actuator_ctrls_ua_qgc = _param_omni_actuator_ua.get();
        _single_modu_cmd_param.actuator_ctrls_us_qgc = _param_omni_actuator_ctrls_us.get();
        _single_modu_cmd_param.actuator_ctrls_pha_qgc = _param_omni_actuator_ctrls_phase.get() * DEG_2_RAD;
        _single_modu_cmd_param.timestamp = hrt_absolute_time();

        _motor_zero_bias = _param_omni_motor_zero_bias.get() * DEG_2_RAD;

        // publish
        _single_modulation_cmd_param_pub.publish(_single_modu_cmd_param);
    }

#if OMNI_TEST_UA_THRUST == 0
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
        _single_modu_cmd_param.actuator_ctrls_ua_qgc += alpha * (target_ua - _single_modu_cmd_param.actuator_ctrls_ua_qgc);
    }

#elif OMNI_TEST_UA_THRUST == 1
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
            stop_increment = true;  // 达到上限后停止
        }
    }
    if (!stop_increment) {
        _single_modu_cmd_param.actuator_ctrls_us_qgc += alpha * (target_us - _single_modu_cmd_param.actuator_ctrls_us_qgc);
    }

#elif OMNI_TEST_UA_THRUST == 2
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

#endif

    _single_modu_cmd_param.timestamp = hrt_absolute_time();

    // publish
    _single_modulation_cmd_param_pub.publish(_single_modu_cmd_param);
}
