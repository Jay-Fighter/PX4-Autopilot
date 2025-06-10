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

VertiqTestInterface::VertiqTestInterface(VertiqSerialInterface* serial_interface, VertiqClientManager* client_manager)
    : _serial_interface(serial_interface),
      _client_manager(client_manager),
      _op_voltage_superposition(_kBroadcastID),
      _op_broadcast_prop_motor_control(_kBroadcastID) {
    client_manager->AddNewClient(&_op_voltage_superposition);
    client_manager->AddNewClient(&_op_broadcast_prop_motor_control);
}

VertiqTestInterface::~VertiqTestInterface() {}

void VertiqTestInterface::run() {}

void VertiqTestInterface::voltage_superposition_test(const vertiq_voltage_superposition_cmd_s& cmd) {
    _vertiq_voltage_superposition_cmd = cmd;

    // set velocity setpoint
    const float ctrl_vel_sp_rad_s = _vertiq_voltage_superposition_cmd.velocity_setpoint * 2.0f * (float)M_PI / 60.0f;  // convert to rad/s
    _op_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface->GetIquartInterface(), ctrl_vel_sp_rad_s);

    //  set amplitude and phase
    _op_voltage_superposition.amplitude_.set(*_serial_interface->GetIquartInterface(), _vertiq_voltage_superposition_cmd.amplitude);
    _op_voltage_superposition.phase_.set(*_serial_interface->GetIquartInterface(), _vertiq_voltage_superposition_cmd.phase);

//     _serial_interface->ProcessSerialTx();
    PX4_INFO("control_velocity_rpm: %f", (double)ctrl_vel_sp_rad_s);
    StartPublishing(&_voltage_superposition_cmd_pub);
}

void VertiqTestInterface::StartPublishing(uORB::Publication<vertiq_voltage_superposition_cmd_s>* voltage_superposition_cmd_pub) {
    _vertiq_voltage_superposition_cmd.timestamp = hrt_absolute_time();
    voltage_superposition_cmd_pub->publish(_vertiq_voltage_superposition_cmd);
}

void VertiqTestInterface::set_vertiq_brake() {
    _op_broadcast_prop_motor_control.ctrl_brake_.set(*_serial_interface->GetIquartInterface());
    _serial_interface->ProcessSerialTx();
}
