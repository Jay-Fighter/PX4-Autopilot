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
#include "voltage_superposition_helper.hpp"
#include <iostream>
#include "perf/perf_counter.h"
#include "px4_platform_common/log.h"
#include "px4_platform_common/module_params.h"
#include "px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp"
#include "px4_platform_common/px4_work_queue/WorkQueueManager.hpp"

using namespace time_literals;
VertiqActuatorHelper::VertiqActuatorHelper()
    : ModuleParams(nullptr), px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default), _brushless_drive_client(0) {
    _port.Open("/dev/ttyUSB0");
    _port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
}

VertiqActuatorHelper::~VertiqActuatorHelper() {
//     if (_port.IsOpen())
//         _port.Close();
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}

int VertiqActuatorHelper::task_spawn(int argc, char** argv) {
    VertiqActuatorHelper* instance = new VertiqActuatorHelper();

    if (instance != nullptr) {

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

int VertiqActuatorHelper::custom_command(int argc, char* argv[]) {
    return print_usage("unknown command");
}

int VertiqActuatorHelper::print_usage(const char* reason) {
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This module is used to help the vertiq actuator to communicate with the board.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME_SIMPLE(MODULE_NAME, "helper");
    return 0;
}

bool VertiqActuatorHelper::init() {
    PX4_INFO("vertiq  actuator helper init...");
    //TODO 注册一些回调
    ScheduleOnInterval(10_ms);
    return true;
}

void VertiqActuatorHelper::Run() {
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
    }
    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);
    _brushless_drive_client.obs_angle_.get(_com);

    uint8_t packet_buf[64];
    uint8_t length = 0;
    // Get the packet from the com interface and place it into the packet buffer
    if (_com.GetTxBytes(packet_buf, length)) {
        // C is a strong typed language -_-
        // so we need to convert to a string buffer to interface with LibSerial
        std::string string_buf((char*)packet_buf, length);

        // Send the get packet request to the motor
        _port.Write(string_buf);
    }
    // Need to wait for the Motor Controller to Respond
    usleep(5000);

    // Serial Receive Buffer
    std::string read_buf;

    // How many bytes are in the read buffer
    length = _port.GetNumberOfBytesAvailable();
    _port.Read(read_buf, length);
    // Again C is strongly types so we have to convert back to byte buffer
    uint8_t* cbuf = (uint8_t*)read_buf.c_str();
    // Transfer the buffer into the com interface
    _com.SetRxBytes(cbuf, length);

    // Temporary Pointer to the packet data location
    uint8_t* rx_data;
    uint8_t rx_data_length;

    // Loads the packet data buffer with data receieved from the motor
    _com.PeekPacket(&rx_data, &rx_data_length);

    // Loads data into the client
    _brushless_drive_client.ReadMsg(rx_data, rx_data_length);

    _com.DropPacket();

    // Let's see if we actually got a response
    if (_brushless_drive_client.obs_angle_.IsFresh()) {
        // Reads the data from the temperature client
        float drive_volts = _brushless_drive_client.obs_angle_.get_reply();
        // (void)drive_volts;
        std::cout << "drive_volts:" << drive_volts << std::endl;
    } else {
        printf("Did not get a response\n");
    }

    perf_end(_loop_perf);
}

extern "C" __EXPORT int voltage_superposition_helper_main(int argc, char* argv[]) {
    return VertiqActuatorHelper::main(argc, argv);
}
