#ifndef OMNISWASHPLESS_H
#define OMNISWASHPLESS_H

#include <cstdint>

#include <float.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/omni_outputs_cmd.h>
#include <uORB/topics/omni_outputs_cmd_param.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#define OMNI_DEBUG 1

#define OMNI_TEST_MODE_SELECTED 3

#define SENSOR_PWM_MAX (8200)
#define SENSOR_PWM_MIN (24)

constexpr uint16_t DSHOT_THROTTLE_MIN = 50;
constexpr uint16_t DSHOT_THROTTLE_MAX = 1800;
#define ACTUATOR_CONTROLS_TO_DSHOT (2000)
#define THROTTLE_MAX (1800)
#define THROTTLE_MIN (0)
#define THROTTLE_SIN_AMP_LIMIT (750)

using time_literals::operator""_s;

class OmniSwashPlateLess : public ModuleBase<OmniSwashPlateLess>, public ModuleParams, public px4::ScheduledWorkItem {
   public:
    OmniSwashPlateLess();
    ~OmniSwashPlateLess();

    /** @see ModuleBase */
    static int task_spawn(int argc, char* argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char* argv[]);

    /** @see ModuleBase */
    static int print_usage(const char* reason = nullptr);

    bool init();

    void Run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

    void modulationCmdCal();

    void limit_and_update_outputs();

    void reset_throttle_output();

    void publish_throttle();

    // for test get params from QGC
    void update_test_params();

   private:
    /*Variable Definition*/
    omni_outputs_cmd_s _single_output_cmd{0};
    omni_outputs_cmd_param_s _single_modu_cmd_param{0};  // Only for QGC test
    int32_t _encoder_rot_dir{0};                            // encoder rotation direction

    /*uORB Subscriber*/
    uORB::Subscription _actuator_output_sub{ORB_ID(actuator_outputs)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    /*uORB Publisher*/
    uORB::Publication<omni_outputs_cmd_s> _single_output_cmd_pub{ORB_ID(omni_outputs_cmd)};
    uORB::Publication<omni_outputs_cmd_param_s> _single_output_cmd_param_pub{ORB_ID(omni_outputs_cmd_param)};

    // Performance (perf) counters
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

    DEFINE_PARAMETERS(
#ifdef OMNI_DEBUG
        (ParamFloat<px4::params::OMNI_UA>)_param_omni_actuator_ctrls_ua, (ParamFloat<px4::params::OMNI_US>)_param_omni_actuator_ctrls_us,
        (ParamFloat<px4::params::OMNI_PHASE>)_param_omni_actuator_ctrls_phase,
        (ParamFloat<px4::params::MOTOR_DELAY_BIAS>)_param_motor_delay_angle_bias
#endif
    )
};
#endif
