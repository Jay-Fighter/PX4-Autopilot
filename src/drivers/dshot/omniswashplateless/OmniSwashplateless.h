#ifndef OMNISWASHPLESS_H
#define OMNISWASHPLESS_H

#include <cstdint>

#include <float.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/pwm_input.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/omni_modulation_cmd.h>
#include <uORB/topics/omni_modulation_cmd_param.h>
#include <uORB/topics/omni_pwm_cap.h>
#include <uORB/topics/omni_motor_telemetry.h>

#define OMNI_DEBUG 1

#define OMNI_TEST_MODE_SELECTED 4

#define SENSOR_PWM_MAX (8200)
#define SENSOR_PWM_MIN (24)

constexpr uint16_t DSHOT_THROTTLE_MIN = 50;
constexpr uint16_t DSHOT_THROTTLE_MAX = 1800;
#define ACTUATOR_CONTROLS_TO_DSHOT (2000)
#define THROTTLE_MAX (1800)
#define THROTTLE_MIN (0)
#define THROTTLE_SIN_AMP_LIMIT (750)

using time_literals::operator""_s;

class OmniSwashPlateLess : public ModuleParams {
   public:
    OmniSwashPlateLess();

    void mortorStateEstimate();

    float motorAngleCal(omni_pwm_cap_s& pwm_input_cap);

    float motorVelocityCal(omni_pwm_cap_s& pwm_input_cap);

    void modulationCmdCal();

    void mix_throttle();

    uint16_t speedCtrl4Dshot(bool on_flag);

    // for test get params from QGC
    void update_test_params();

   private:
    /*Variable Definition*/
    pwm_input_s pwm_input_cap_data{0};
    omni_modulation_cmd_s _single_modu_cmd{0};
    omni_modulation_cmd_param_s _single_modu_cmd_param{0};  // Only for QGC test
    omni_pwm_cap_s _pwm_input_cap{0};                       // pwm cap data from ORB_ID(pwm_input)
    omni_motor_telemetry_s _motor_telemetry{0};
    float _motor_zero_bias{0.0f};  // rad, motor zero bias, used for motor angle calibration
    int32_t _encoder_rot_dir{0};   // encoder rotation direction
    float _motor_delay_angle_bias_rad{0.0f};

    /*uORB Subscriber*/
    uORB::Subscription pwm_input_sub{ORB_ID(pwm_input)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    /*uORB Publisher*/
    uORB::Publication<omni_modulation_cmd_s> _single_modulation_cmd_pub{ORB_ID(omni_modulation_cmd)};
    uORB::Publication<omni_modulation_cmd_param_s> _single_modulation_cmd_param_pub{ORB_ID(omni_modulation_cmd_param)};
    uORB::Publication<omni_pwm_cap_s> _single_pwm_input_cap_pub{ORB_ID(omni_pwm_cap)};
    uORB::Publication<omni_motor_telemetry_s> _motor_telemetry_pub{ORB_ID(omni_motor_telemetry)};

    DEFINE_PARAMETERS(
#ifdef OMNI_DEBUG
        (ParamFloat<px4::params::MOTOR_ZERO_BIAS>)_param_omni_motor_zero_bias, (ParamFloat<px4::params::OMNI_UA>)_param_omni_actuator_ctrls_ua,
        (ParamFloat<px4::params::OMNI_US>)_param_omni_actuator_ctrls_us, (ParamFloat<px4::params::OMNI_PHASE>)_param_omni_actuator_ctrls_phase,
        (ParamInt<px4::params::ENCODER_ROT_DIR>)_param_encoder_rot_dir, (ParamFloat<px4::params::MOTOR_DELAY_BIAS>)_param_motor_delay_angle_bias
#endif
    )
};
#endif
