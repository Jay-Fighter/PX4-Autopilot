/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

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

class VertiqActuatorHelper : public ModuleBase<VertiqActuatorHelper>, public ModuleParams, public px4::ScheduledWorkItem {
   public:
    VertiqActuatorHelper();
    ~VertiqActuatorHelper() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char* argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char* argv[]);

    /** @see ModuleBase */
    static int print_usage(const char* reason = nullptr);

    bool init();

   private:
    LibSerial::SerialPort _port;

    void Run() override;

    // Performance (perf) counters
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
    // Performance (perf) counters
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

    //Vertiq client
    GenericInterface _com;
    BrushlessDriveClient _brushless_drive_client;
};
