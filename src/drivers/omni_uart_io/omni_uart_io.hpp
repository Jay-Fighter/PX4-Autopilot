/**
 * @file omni_uart_io.hpp
 * @author {jayjie} ({zhengyongjie@sia.cn})
 * @brief
 * @version 0.1
 * @date 2025-09-25
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
#include <px4_log.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <px4_getopt.c>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

class OmniSerialInterface : public ModuleBase<OmniSerialInterface>, public ModuleParams, public px4::ScheduledWorkItem {
   public:
    OmniSerialInterface(const char* uart_device);
    ~OmniSerialInterface();

    void Run() override;

    bool init(const char* uart_device);

    /**
     * @brief Initialize our serial peripheral
     */
    int InitSerial(const char* uart_device);

    /**
     * Turn off and close the serial connection
     */
    void DeinitSerial();

    /**
     * set the Baudrate
     * @param baud
     * @return 0 on success, <0 on error
     */
    int ConfigureSerialPeripheral(unsigned baud);

    /**
     * @brief Check to see if there are any valid packets for us to read
     *
     * @return true If there is a packet
     * @return false If there is not a packet
     */
    bool CheckForRx();

    /**
     * @brief Read a packet from our packet finder and return a pointer to the beginning of the data
     *
     * @return uint8_t* A pointer to the start of the packet
     */
    uint8_t* ReadAndSetRxBytes();

    /**
     * @brief check to see if there is any data that we need to transmit over serial
     */
    void ProcessSerialTx();

    void print_info();

    void ReOpenSerial();

   private:
    char _port_in_use[20]{};
    uint8_t _bytes_available;
    // Buffers for data to transmit or that we're receiving
    uint8_t _rx_buf[256];
    uint8_t _tx_buf[256];

    // The port that we're using for communication
    int _uart_fd{-1};

    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
    perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": update interval")};

    // QGC param
    DEFINE_PARAMETERS(

        (ParamInt<px4::params::OMNI_UART_BAUD>)_param_omni_uart_baud

    )
};
