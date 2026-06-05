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

#include "ActuatorEffectivenessOmniVirtualForce.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

using namespace matrix;

// add by jayjie
ActuatorEffectivenessOmniVirtualForce::ActuatorEffectivenessOmniVirtualForce(ModuleParams *parent) :
	ModuleParams(parent),
	_mc_rotors(this)
{
}

bool ActuatorEffectivenessOmniVirtualForce::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	const ActuatorEffectivenessRotors::Geometry &geometry = _mc_rotors.geometry();

	if (geometry.num_rotors != NUM_PHYSICAL_MOTORS) {
		PX4_ERR("OmniVirtualForce requires 4 rotors, got %d", geometry.num_rotors);
		return false;
	}

	configuration.num_actuators_matrix[configuration.selected_matrix] = 0;
	EffectivenessMatrix &effectiveness = configuration.effectiveness_matrices[configuration.selected_matrix];

	for (int row = 0; row < NUM_AXES; row++) {
		for (int column = 0; column < NUM_ACTUATORS; column++) {
			effectiveness(row, column) = 0.f;
		}
	}

	for (int motor_index = 0; motor_index < NUM_PHYSICAL_MOTORS; motor_index++) {
		const int base = motor_index * AXES_PER_MOTOR;
		const float ct = geometry.rotors[motor_index].thrust_coef;

		// Virtual force variables are [Fx_i, Fy_i, Fz_i]. Positive u_fz produces negative body-Z thrust.
		effectiveness(ControlAxis::THRUST_X, base + 0) = ct;
		effectiveness(ControlAxis::THRUST_Y, base + 1) = ct;
		effectiveness(ControlAxis::THRUST_Z, base + 2) = -ct;
	}

	for (int motor_index = 0; motor_index < NUM_PHYSICAL_MOTORS; motor_index++) {
		const int base = motor_index * AXES_PER_MOTOR;
		Vector3f axis = geometry.rotors[motor_index].axis;
		const float axis_norm = axis.norm();

		if (axis_norm <= FLT_EPSILON) {
			PX4_ERR("Invalid rotor axis %d", motor_index);
			return false;
		}

		axis /= axis_norm;

		const Vector3f &position = geometry.rotors[motor_index].position;
		const float ct = geometry.rotors[motor_index].thrust_coef;
		const float km = geometry.rotors[motor_index].moment_ratio;

		// r x F with z_i = 0 and Fz_body = -ct * u_fz.
		effectiveness(ControlAxis::ROLL, base + 2) = -position(1) * ct;
		effectiveness(ControlAxis::PITCH, base + 2) = position(0) * ct;

		// Keep yaw authority equivalent to a normal quad: horizontal virtual forces do not create yaw.
		effectiveness(ControlAxis::YAW, base + 2) = -ct * km * axis(2);
	}

	configuration.actuatorsAdded(ActuatorType::MOTORS, NUM_VIRTUAL_ACTUATORS);

	return true;
}

void ActuatorEffectivenessOmniVirtualForce::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	omni_actuator_setpoint_s omni_actuator_sp{};

	omni_actuator_sp.torque_sp_body[0] = control_sp(ControlAxis::ROLL);
	omni_actuator_sp.torque_sp_body[1] = control_sp(ControlAxis::PITCH);
	omni_actuator_sp.torque_sp_body[2] = control_sp(ControlAxis::YAW);

	omni_actuator_sp.thrust_sp_body[0] = control_sp(ControlAxis::THRUST_X);
	omni_actuator_sp.thrust_sp_body[1] = control_sp(ControlAxis::THRUST_Y);
	omni_actuator_sp.thrust_sp_body[2] = control_sp(ControlAxis::THRUST_Z);

	ActuatorVector actuator_sp_clipped = actuator_sp;

	for (int i = 0; i < NUM_VIRTUAL_ACTUATORS; i++) {
		actuator_sp_clipped(i) = math::constrain(actuator_sp_clipped(i), actuator_min(i), actuator_max(i));
		omni_actuator_sp.control[i] = actuator_sp_clipped(i);
	}

	for (int motor_index = 0; motor_index < NUM_PHYSICAL_MOTORS; motor_index++) {
		const int base = motor_index * AXES_PER_MOTOR;
		omni_actuator_sp.fx[motor_index] = actuator_sp_clipped(base + 0);
		omni_actuator_sp.fy[motor_index] = actuator_sp_clipped(base + 1);
		omni_actuator_sp.fz[motor_index] = actuator_sp_clipped(base + 2);
		omni_actuator_sp.f[motor_index] = sqrtf(omni_actuator_sp.fx[motor_index] * omni_actuator_sp.fx[motor_index]
							+ omni_actuator_sp.fy[motor_index] * omni_actuator_sp.fy[motor_index]
							+ omni_actuator_sp.fz[motor_index] * omni_actuator_sp.fz[motor_index]);
		omni_actuator_sp.num_groups++;
	}

	omni_actuator_sp.timestamp = hrt_absolute_time();
	_omni_actuator_setpoint_pub.publish(omni_actuator_sp);
}
// end
