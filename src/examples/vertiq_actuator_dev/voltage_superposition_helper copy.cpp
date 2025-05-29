/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/log.h>
#include <unistd.h>
#include <cstdint>

#include "libserial/SerialPort.h"
#include "libserial/SerialStream.h"

#include <cstdio>
#include <iostream>

// vertiq include files
#include "brushless_drive_client.hpp"
#include "generic_interface.hpp"
#include "propeller_motor_control_client.hpp"
#include "voltage_superposition_client.hpp"

using namespace LibSerial;

extern "C" __EXPORT int voltage_superposition_helper_main(int argc, char* argv[]);

void send_packet(GenericInterface& com, SerialPort& port) {
    uint8_t buf[64];
    uint8_t len = 0;
    if (com.GetTxBytes(buf, len)) {
        port.Write(std::string(reinterpret_cast<char*>(buf), len));
    }
}

int voltage_superposition_helper_main(int argc, char* argv[]) {
    PX4_INFO("PX4: voltage superposition helper start.");

    SerialPort port("/dev/ttyUSB0");
    port.SetBaudRate(BaudRate::BAUD_115200);

    GenericInterface com;
    PropellerMotorControlClient pmc_client(0);

    for (int i = 0; i < 7; ++i) {
        PX4_INFO("PX4: voltage superposition set velocity mode.");
        pmc_client.ctrl_mode_.set(com, 4);          // velocity mode
        pmc_client.ctrl_velocity_.set(com, 300.f);  // rad/s
        send_packet(com, port);
        usleep(5000);
    }
    getchar();
    PX4_INFO("PX4: voltage superposition helper stop.");
    while (true) {
        pmc_client.ctrl_brake_.set(com);  // brake
        send_packet(com, port);
        usleep(5000);
    }

    return OK;
}

extern "C" __EXPORT int voltage_superposition_helper_main(int argc, char* argv[]) {
    return VoltageSuperpositionHelper::main(argc, argv);
}
