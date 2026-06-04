/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessOmniMultirotor.hpp"
#include "px4_platform_common/log.h"

using namespace matrix;

ActuatorEffectivenessOmniMultirotor::ActuatorEffectivenessOmniMultirotor(ModuleParams* parent) : ModuleParams(parent), _mc_rotors(this) {}

bool ActuatorEffectivenessOmniMultirotor::getEffectivenessMatrix(Configuration& configuration, EffectivenessUpdateReason external_update) {
        if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
                return false;
        }

        const ActuatorEffectivenessRotors::Geometry& geometry = _mc_rotors.geometry();
        const bool motors_added_successfully = _mc_rotors.addActuators(configuration);

        // 12 actuator channels: M1=[Fx,Fy,Fz], M2=[Fx,Fy,Fz] ... M4
        configuration.num_actuators_matrix[0] = 12;
        EffectivenessMatrix& effectiveness = configuration.effectiveness_matrices[0];

        // Clear matrix
        for (int r = 0; r < NUM_AXES; r++) {
                for (int c = 0; c < NUM_ACTUATORS; c++) {
                        effectiveness(r, c) = 0.f;
                }
        }

        // Fill force mapping Fx,Fy,Fz for each motor
        for (int index = 0; index < 4; index++) {

                float ct = geometry.rotors[index].thrust_coef;
                effectiveness(3, index * 3 + 0) = ct;  // Fx
                effectiveness(4, index * 3 + 1) = ct;  // Fy
                effectiveness(5, index * 3 + 2) = -ct;  // Fz
        }

        // Fill moment = r × F
        for (int index = 0; index < 4; index++) {

                // Get rotor axis
                Vector3f axis = geometry.rotors[index].axis;

                // Normalize axis
                float axis_norm = axis.norm();

                if (axis_norm > FLT_EPSILON) {
                        axis /= axis_norm;

                } else {
                        // Bad axis definition, ignore this rotor
                        continue;
                }

                // Get coefficients
                float ct = geometry.rotors[index].thrust_coef;
                float km = geometry.rotors[index].moment_ratio;

                // Get rotor position
                const Vector3f& position = geometry.rotors[index].position;

                // moment wrt Fx,Fy,Fz column index
                int base = index * 3;

                effectiveness(0, base + 2) = -position(1) * ct;  // Mx from Fz

                effectiveness(1, base + 2) = position(0) * ct;  // My from Fz

                // effectiveness(2, base + 0) = -position(1) * ct;  // Mz from Fx
                // effectiveness(2, base + 1) = position(0) * ct;   // Mz from Fy
                effectiveness(2, base + 2) = -ct * km * axis(2);  // Mz from Fz
        }
        // configuration.actuatorsAdded(ActuatorType::MOTORS, 12);

        return motors_added_successfully;
}

void ActuatorEffectivenessOmniMultirotor::updateSetpoint(const matrix::Vector<float, NUM_AXES>& control_sp, int matrix_index,
                                                         ActuatorVector& actuator_sp, const matrix::Vector<float, NUM_ACTUATORS>& actuator_min,
                                                         const matrix::Vector<float, NUM_ACTUATORS>& actuator_max) {
        omni_actuator_setpoint_s omni_actuator_sp{0};

	// copy desired torque/thrust body to omni_actuator_sp
	omni_actuator_sp.torque_sp_body[0] = control_sp(ControlAxis::ROLL);
	omni_actuator_sp.torque_sp_body[1] = control_sp(ControlAxis::PITCH);
	omni_actuator_sp.torque_sp_body[2] = control_sp(ControlAxis::YAW);

	omni_actuator_sp.thrust_sp_body[0] = control_sp(ControlAxis::THRUST_X);
	omni_actuator_sp.thrust_sp_body[1] = control_sp(ControlAxis::THRUST_Y);
	omni_actuator_sp.thrust_sp_body[2] = control_sp(ControlAxis::THRUST_Z);

        for (int index = 0; index < 4; index++) {
                int base = index * 3;
                omni_actuator_sp.control[base + 0] = actuator_sp(base + 0);
                omni_actuator_sp.control[base + 1] = actuator_sp(base + 1);
                omni_actuator_sp.control[base + 2] = actuator_sp(base + 2);

                omni_actuator_sp.fx[index] = actuator_sp(base + 0);
                omni_actuator_sp.fy[index] = actuator_sp(base + 1);
                omni_actuator_sp.fz[index] = actuator_sp(base + 2);
                omni_actuator_sp.f[index] =
                    sqrtf(omni_actuator_sp.fx[index] * omni_actuator_sp.fx[index] + omni_actuator_sp.fy[index] * omni_actuator_sp.fy[index] +
                          omni_actuator_sp.fz[index] * omni_actuator_sp.fz[index]);
                omni_actuator_sp.num_groups++;
                // PX4_INFO("[OmniMotor%d] Fx=%.3f Fy=%.3f Fz=%.3f", index + 1, (double)Fx, (double)Fy, (double)Fz);
        }
        omni_actuator_sp.timestamp = hrt_absolute_time();
        _omni_actuator_setpoint_pub.publish(omni_actuator_sp);
}
