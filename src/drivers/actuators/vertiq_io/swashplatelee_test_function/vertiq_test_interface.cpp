/**
 * @file vertiq_test_interface.cpp
 * @author {jayjie} ({zhengyongjie@sia.cn})
 * @brief
 * @version 0.1
 * @date 2025-06-09
 *
 * @copyright Copyright (c) 2025 {jayjie}.
 *
 * This program is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 *
 */
#include "vertiq_test_interface.hpp"
#include <unistd.h>
#include "commander/HealthAndArmingChecks/Common.hpp"
#include "px4_platform_common/log.h"

VertiqTestInterface::VertiqTestInterface(VertiqSerialInterface* serial_interface, VertiqClientManager* client_manager)
    : ModuleParams(nullptr),
      _is_new_cmd(false),
      _serial_interface(serial_interface),
      _client_manager(client_manager),
      _op_voltage_superposition(_kBroadcastID),
      _op_broadcast_prop_motor_control(_kBroadcastID) {
    client_manager->AddNewClient(&_op_voltage_superposition);
    client_manager->AddNewClient(&_op_broadcast_prop_motor_control);
}

VertiqTestInterface::~VertiqTestInterface() {}

void VertiqTestInterface::run() {}

void VertiqTestInterface::parameters_update() {
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();

        SetVelocityKp();

        SetVelocityKi();

        SetVelocityKd();
    }
}

void VertiqTestInterface::voltage_superposition_test(const vertiq_voltage_superposition_cmd_s& cmd) {
    parameters_update();

    _vertiq_voltage_superposition_cmd = cmd;

    // set velocity setpoint
    const float ua_rpm = _vertiq_voltage_superposition_cmd.velocity_setpoint;
    const float us_rpm = _vertiq_voltage_superposition_cmd.amplitude;
    const float phase = _vertiq_voltage_superposition_cmd.phase;

    // calculate the sinusoidal angular frequency
    const float omega = 2.0f * static_cast<float>(M_PI) * _vertiq_voltage_superposition_cmd.velocity_setpoint / 60.0f;

    // calculate the total velocity
    if (!_is_new_cmd) {
        last_swashplateless_cmd_update = hrt_absolute_time();
        _is_new_cmd = true;
    }
    float t_s = (float)(hrt_absolute_time() - last_swashplateless_cmd_update) * 1.0e-6f;
    float u = 0.0f;
    if (t_s < 1.0f) {
        u = ua_rpm * t_s * 2.0f * static_cast<float>(M_PI) / 60.0f;
    } else {
        u = (ua_rpm + us_rpm * (float)cos(omega * t_s - phase)) * 2.0f * static_cast<float>(M_PI) / 60.0f;
    }

    // publish the velocity cmd
    _vertiq_voltage_superposition_cmd.velocity_setpoint = u * 60.0f / (2 * static_cast<float>(M_PI));
    _vertiq_voltage_superposition_cmd.amplitude = _vertiq_voltage_superposition_cmd.amplitude;
    _vertiq_voltage_superposition_cmd.phase = _vertiq_voltage_superposition_cmd.phase;
    StartPublishing(&_voltage_superposition_cmd_pub);

    // send the velocity to the vertiq
    _op_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface->GetIquartInterface(), u);
    //     _serial_interface->ProcessSerialTx();
}

void VertiqTestInterface::StartPublishing(uORB::Publication<vertiq_voltage_superposition_cmd_s>* voltage_superposition_cmd_pub) {
    _vertiq_voltage_superposition_cmd.timestamp = hrt_absolute_time();
    voltage_superposition_cmd_pub->publish(_vertiq_voltage_superposition_cmd);
}

void VertiqTestInterface::set_vertiq_brake() {
    _op_broadcast_prop_motor_control.ctrl_brake_.set(*_serial_interface->GetIquartInterface());
    _serial_interface->ProcessSerialTx();
}

void VertiqTestInterface::SetVelocityKp() {
    float kp = _param_vertiq_vel_kp.get();
    _op_broadcast_prop_motor_control.velocity_kp_.set(*_serial_interface->GetIquartInterface(), kp);
    //     _op_broadcast_prop_motor_control.velocity_kp_.save(*_serial_interface->GetIquartInterface());
    PX4_INFO("kp%.7f", (double)kp);
}

void VertiqTestInterface::SetVelocityKi() {
    float ki = _param_vertiq_vel_ki.get();
    _op_broadcast_prop_motor_control.velocity_ki_.set(*_serial_interface->GetIquartInterface(), ki);
    //     _op_broadcast_prop_motor_control.velocity_ki_.save(*_serial_interface->GetIquartInterface());
    PX4_INFO("ki%.7f", (double)ki);
}
void VertiqTestInterface::SetVelocityKd() {
    float kd = _param_vertiq_vel_kd.get();
    _op_broadcast_prop_motor_control.velocity_kd_.set(*_serial_interface->GetIquartInterface(), kd);
    //     _op_broadcast_prop_motor_control.velocity_kd_.save(*_serial_interface->GetIquartInterface());
    PX4_INFO("kd%.7f", (double)kd);
}
