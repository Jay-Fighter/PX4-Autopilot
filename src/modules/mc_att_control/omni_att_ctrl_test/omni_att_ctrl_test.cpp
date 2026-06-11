/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

// add by jayjie
#include <drivers/drv_hrt.h>
#include <inttypes.h>
#include <lib/perf/perf_counter.h>
#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
// add by jayjie
#include <uORB/topics/vehicle_attitude.h>
// end
#include <uORB/topics/vehicle_attitude_setpoint.h>

using namespace time_literals;

class OmniAttCtrlTest : public ModuleBase<OmniAttCtrlTest>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	OmniAttCtrlTest();
	~OmniAttCtrlTest() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void update_schedule();
	// add by jayjie
	bool update_yaw_hold();
	// end
	void publish_attitude_setpoint();

	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	// add by jayjie
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	// end
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	hrt_abstime _publish_interval_us{10000};
	uint32_t _publish_count{0};
	// add by jayjie
	bool _yaw_hold_valid{false};
	float _yaw_hold_rad{0.f};
	// end

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OMNI_ATT_EN>) _param_omni_att_en,
		(ParamFloat<px4::params::OMNI_ATT_ROLL>) _param_omni_att_roll,
		(ParamFloat<px4::params::OMNI_ATT_PITCH>) _param_omni_att_pitch,
		(ParamFloat<px4::params::OMNI_ATT_YAW>) _param_omni_att_yaw,
		(ParamFloat<px4::params::OMNI_ATT_THR_Z>) _param_omni_att_thr_z,
		(ParamFloat<px4::params::OMNI_ATT_RATE>) _param_omni_att_rate
	)
};

OmniAttCtrlTest::OmniAttCtrlTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

OmniAttCtrlTest::~OmniAttCtrlTest()
{
	perf_free(_loop_perf);
}

bool OmniAttCtrlTest::init()
{
	updateParams();
	// add by jayjie
	update_yaw_hold();
	// end
	update_schedule();
	return true;
}

void OmniAttCtrlTest::update_schedule()
{
	const float publish_rate_hz = math::constrain(_param_omni_att_rate.get(), 1.f, 200.f);
	const hrt_abstime interval_us = static_cast<hrt_abstime>(1000000.f / publish_rate_hz);

	if (interval_us != _publish_interval_us) {
		_publish_interval_us = interval_us;
		ScheduleOnInterval(_publish_interval_us);

	} else {
		ScheduleOnInterval(_publish_interval_us);
	}
}

// add by jayjie
bool OmniAttCtrlTest::update_yaw_hold()
{
	if (_yaw_hold_valid) {
		return true;
	}

	vehicle_attitude_s vehicle_attitude{};

	if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
		const matrix::Quatf q_att(vehicle_attitude.q);
		const matrix::Eulerf euler_att(q_att);
		_yaw_hold_rad = euler_att.psi();
		_yaw_hold_valid = true;
		return true;
	}

	return false;
}
// end

void OmniAttCtrlTest::publish_attitude_setpoint()
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	const float roll = math::radians(_param_omni_att_roll.get());
	const float pitch = math::radians(_param_omni_att_pitch.get());
	// add by jayjie
	const float yaw = _yaw_hold_rad;
	// end

	const matrix::Quatf q_sp{matrix::Eulerf{roll, pitch, yaw}};
	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.yaw_sp_move_rate = 0.f;
	attitude_setpoint.thrust_body[0] = 0.f;
	attitude_setpoint.thrust_body[1] = 0.f;
	attitude_setpoint.thrust_body[2] = math::constrain(_param_omni_att_thr_z.get(), -1.f, 0.f);
	attitude_setpoint.reset_integral = false;
	attitude_setpoint.fw_control_yaw_wheel = false;
	attitude_setpoint.timestamp = hrt_absolute_time();

	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
	_publish_count++;
}

void OmniAttCtrlTest::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		update_schedule();
	}

	// add by jayjie
	update_yaw_hold();
	// end

	if (_param_omni_att_en.get() != 0 && _yaw_hold_valid) {
		publish_attitude_setpoint();
	}

	perf_end(_loop_perf);
}

int OmniAttCtrlTest::task_spawn(int argc, char *argv[])
{
	OmniAttCtrlTest *instance = new OmniAttCtrlTest();

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

int OmniAttCtrlTest::print_status()
{
	// add by jayjie
	const float yaw_status_deg = _yaw_hold_valid ? math::degrees(_yaw_hold_rad) : _param_omni_att_yaw.get();
	// end

	PX4_INFO("enabled: %d", static_cast<int>(_param_omni_att_en.get()));
	PX4_INFO("rpy deg: roll %.2f pitch %.2f yaw %.2f",
		 static_cast<double>(_param_omni_att_roll.get()),
		 static_cast<double>(_param_omni_att_pitch.get()),
		 static_cast<double>(yaw_status_deg));
	PX4_INFO("thrust_z: %.3f rate: %.1f Hz published: %" PRIu32,
		 static_cast<double>(_param_omni_att_thr_z.get()),
		 static_cast<double>(_param_omni_att_rate.get()),
		 _publish_count);
	perf_print_counter(_loop_perf);
	return 0;
}

int OmniAttCtrlTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int OmniAttCtrlTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publishes vehicle_attitude_setpoint for omni attitude-loop testing.
The desired attitude is configured by OMNI_ATT_ROLL, OMNI_ATT_PITCH, and OMNI_ATT_YAW in degrees.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("omni_att_ctrl_test", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int omni_att_ctrl_test_main(int argc, char *argv[])
{
	return OmniAttCtrlTest::main(argc, argv);
}
// end
