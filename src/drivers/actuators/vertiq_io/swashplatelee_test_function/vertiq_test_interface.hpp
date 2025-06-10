/**
 * @file vertiq_test_interface.hpp
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

#pragma once
#include <uORB/topics/vertiq_voltage_superposition_cmd.h>
#include <uORB/Publication.hpp>

#include "drivers/actuators/vertiq_io/vertiq_client_manager.hpp"
#include "../vertiq_client_manager.hpp"
#include "../vertiq_serial_interface.hpp"
#include "../iq-module-communication-cpp/inc/voltage_superposition_client.hpp"

class VertiqTestInterface {
   public:
    VertiqTestInterface(VertiqSerialInterface* serial_interface, VertiqClientManager* client_manager);
    ~VertiqTestInterface();

    void set_vertiq_brake();

    /**
     * @brief  Run the test
     */
    void run();

    /**
     * @brief   Voltage superposition test
     */
    void voltage_superposition_test(const vertiq_voltage_superposition_cmd_s& cmd);

    /**
     * @brief publisher for vertiq_voltage_superposition_cmd
     * @param voltage_superposition_cmd_pub
     */
    void StartPublishing(uORB::Publication<vertiq_voltage_superposition_cmd_s>* voltage_superposition_cmd_pub);

   private:
    VertiqSerialInterface* _serial_interface;
    VertiqClientManager* _client_manager;

    // vertiq clients
    VoltageSuperPositionClient _op_voltage_superposition;
    PropellerMotorControlClient _op_broadcast_prop_motor_control;

    // uORB publications
    uORB::Publication<vertiq_voltage_superposition_cmd_s> _voltage_superposition_cmd_pub{ORB_ID(vertiq_voltage_superposition_cmd)};

    // cmd
    vertiq_voltage_superposition_cmd_s _vertiq_voltage_superposition_cmd{};
};
