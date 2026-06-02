#ifndef OMNISWASHPLESS_H
#define OMNISWASHPLESS_H

#include <cstdint>

#include <float.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <stddef.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_test.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/omni_outputs_cmd_groups.h>
#include <uORB/topics/omni_outputs_cmd.h>
#include <uORB/topics/omni_outputs_cmd_param.h>
#include <uORB/topics/omni_actuator_setpoint.h>
// add by jayjie
#include <uORB/topics/manual_control_setpoint.h>
// end

#include "../omni_common/omni_debug.h"

/* Test7: Fixed ua, us, triangle wave on phase (0 → 360 → 0) until count > cycles_target
 * OMNI_TEST_UA_RAMP_US_0           = 0,  // Test1: UA ramp up (0→max), US = 0
 * OMNI_TEST_US_RAMP_UA_FIXED       = 1,  // Test2: US ramp up (0→max), UA = constant
 * OMNI_TEST_UA_RAMP_US_FIXED       = 2,  // Test3: UA ramp up (0→max), US = constant
 * OMNI_TEST_PHASE_SWEEP_FIXED_UAUS = 3,  // Test4: Phase rotates continuously, UA & US = constant
 * OMNI_TEST_DELAY_US_AFTER_UA      = 4,  // Test5: First apply UA, then add US after delay
 * OMNI_TEST_TRI_US_FIXED_UA_PHASE  = 5,  // Test6: Triangle wave on US (0→max→0), UA & Phase fixed
 * OMNI_TEST_TRI_PHASE_FIXED_UA_US  = 6,  // Test7: Triangle wave on Phase (0°→360°→0°), UA & US fixed
 * OMNI_TEST_TRI_PHASE_FIXED_UA_US  = 10,  // Test10: Triangle wave on Phase (0°→360°→0°), UA & US fixed
 */

#define OMNI_TEST_MODE_SELECTED 4

#define OMNI_ACTUATOR_NUM 4

constexpr uint16_t DSHOT_THROTTLE_MIN = 50;
constexpr uint16_t DSHOT_THROTTLE_MAX = 1800;
constexpr float us_2_ua_ratio_max = 0.25;
#define ACTUATOR_CONTROLS_TO_DSHOT (2000)
#define THROTTLE_MAX (1300)
#define THROTTLE_MIN (0)
#define THROTTLE_SIN_AMP_LIMIT (400)

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

        void limit_and_update_outputs(omni_actuator_setpoint_s& output);

        void limit_and_update_outputs();

        // add by jayjie
        void limit_and_update_outputs(const manual_control_setpoint_s& manual_control_setpoint);
        // end

        void reset_throttle_output();

        void publish_throttle();

        // for test get params from QGC
        void update_test_params();

       private:
        // add by jayjie
        float getMotorPhaseFromBodyPhase(float phase_body, size_t motor_index) const;
        // end

        /*Variable Definition*/
        omni_outputs_cmd_s _single_output_cmd{0};
        omni_outputs_cmd_param_s _single_modu_cmd_param{0};  // Only for QGC test
        omni_outputs_cmd_groups_s _omni_outputs_cmd_groups{0};
        int32_t _encoder_rot_dir{0};  // encoder rotation direction

        /*uORB Subscriber*/
        uORB::Subscription _actuator_output_sub{ORB_ID(actuator_test)};
        uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
        uORB::Subscription _omni_actuator_setpoint_sub{ORB_ID(omni_actuator_setpoint)};
        // add by jayjie
        uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
        // end

        /*uORB Publisher*/
        uORB::Publication<omni_outputs_cmd_s> _single_output_cmd_pub{ORB_ID(omni_outputs_cmd)};
        uORB::Publication<omni_outputs_cmd_param_s> _single_output_cmd_param_pub{ORB_ID(omni_outputs_cmd_param)};
        uORB::Publication<omni_outputs_cmd_groups_s> _omni_outputs_cmd_groups_pub{ORB_ID(omni_outputs_cmd_groups)};

        // Performance (perf) counters
        perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
        perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

        // exp flags
        bool _stop_increment{false};
        hrt_abstime _last_increment_time{0};  // 时间记录
        float _target_ua{0.0f};
        float _target_us{0.0f};
        float _target_phase{0.0f};

        DEFINE_PARAMETERS((ParamFloat<px4::params::OMNI_UA>)_param_omni_actuator_ctrls_ua,
                          (ParamFloat<px4::params::OMNI_US>)_param_omni_actuator_ctrls_us,
                          (ParamFloat<px4::params::OMNI_PHASE>)_param_omni_actuator_ctrls_phase,
                          (ParamFloat<px4::params::MOTOR_DELAY_BIAS>)_param_motor_delay_angle_bias,
                          // add by jayjie
                          (ParamInt<px4::params::OMNI_RC_TEST_IDX>)_param_omni_rc_test_index,
                          (ParamInt<px4::params::OMNI_RC_TST_MODE>)_param_omni_rc_test_mode
                          // end
        )
};
#endif
