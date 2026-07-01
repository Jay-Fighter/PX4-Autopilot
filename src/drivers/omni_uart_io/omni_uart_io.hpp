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
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/omni_motor_telemetry.h>
#include <uORB/topics/omni_motors_telemetry.h>
#include <uORB/topics/omni_outputs_cmd.h>
#include <uORB/topics/omni_outputs_cmd_groups.h>
#include <uORB/topics/omni_outputs_cmd_frame.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>

#include "../omni_common/omni_debug.h"
#include <uORB/topics/parameter_update.h>

#include <containers/Array.hpp>

// Frame packet definition
#define FRAME_HEADER_1 0xAB
#define FRAME_HEADER_2 0xCD
#define FRAME_END_1 0x0D
#define FRAME_END_2 0x0A
#define FRAME_OUTER_TX_HEADER_1 0x55
#define FRAME_OUTER_TX_HEADER_2 0xAA
#define FRAME_OUTER_TX_END_1 0xAA
#define FRAME_OUTER_TX_END_2 0x55
#define FRAME_OUTER_RX_HEADER_1 0x55
#define FRAME_OUTER_RX_HEADER_2 0xBB
#define FRAME_OUTER_RX_END_1 0xBB
#define FRAME_OUTER_RX_END_2 0x55
#define MOTOR_INIT_DISABLED 0x00  // 电机未进行初始化设置
#define MOTOR_INIT_ENABLED 0x01   // 电机初始化设置完成

// Packet length based on the protocol defined
const ssize_t FRAME_LEN_RX = 31;  // single sub-frame
const ssize_t FRAME_LEN_TX = 21;  // single sub-frame
const ssize_t FRAME_LEN_OUTER_TX = 2 + (4 * FRAME_LEN_TX) + 1 + 2;
const ssize_t FRAME_LEN_OUTER_RX = 2 + (4 * FRAME_LEN_RX) + 1 + 2;

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
        void ProcessSerialRx();

        float bytesToFloat(const uint8_t* bytes);

        uint16_t bytesToUint16(const uint8_t* bytes);

        /**
         * @brief check to see if there is any data that we need to transmit over serial
         */
        void ProcessSerialTx();

        /**
         * @brief Calculate the checksum for a given packet
         */
        uint8_t calcChecksum(const uint8_t* data, size_t len);

        /**
         * @brief Pack a throttle command packet
         */
        void packThrottleCmd(const omni_outputs_cmd_s& packet, uint8_t encoder_reverse_flag, uint8_t* frame, uint8_t& frame_len);

        /**
         * @brief Pack a 4-motor grouped packet (outer frame)
         */
        void packThrottleCmdGroup(const omni_outputs_cmd_groups_s& packet, uint8_t* frame, uint8_t& frame_len);

        /**
         * @brief Parse a single sub-frame (motor telemetry)
         */
        bool parseSingleRxFrame(const uint8_t* frame, bool check_checksum, omni_motor_telemetry_s* out, bool publish_single);

        void print_info();

        void ReOpenSerial();

        void setMotorZeroPosAndRev();

        // add by jayjie
        uint8_t getEncoderReverseFlagForMotor(uint8_t motor_index) const;
        // end

        void PublishEscStatusFromOmniTelemetry(const omni_motors_telemetry_s& motors_telemetry);

       private:
        char _port_in_use[20]{};
        // add by jayjie
        int _bytes_available{0};
        // end

        // Buffers for data to transmit or that we're receiving
        uint8_t _rx_buf[256];
        uint8_t _tx_buf[256];
        // add by jayjie
        static constexpr size_t RX_ACCUM_BUF_LEN = 512;
        uint8_t _rx_accum[RX_ACCUM_BUF_LEN]{};
        size_t _rx_accum_len{0};
        // end

        // The port that we're using for communication
        int _uart_fd{-1};

        omni_motor_telemetry_s _motor_telemetry{0};

        /*uORB Subscriber*/
        uORB::Subscription _single_output_cmd_sub{ORB_ID(omni_outputs_cmd)};
        uORB::Subscription _output_cmd_groups_sub{ORB_ID(omni_outputs_cmd_groups)};
        uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
        uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

        /*uORB Publisher*/
        uORB::Publication<omni_motor_telemetry_s> _motor_telemetry_pub{ORB_ID(omni_motor_telemetry)};
        uORB::Publication<omni_motors_telemetry_s> _motors_telemetry_pub{ORB_ID(omni_motors_telemetry)};
        uORB::Publication<omni_outputs_cmd_frame_s> _omni_outputs_cmd_frame_pub{ORB_ID(omni_outputs_cmd_frame)};
        uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
        uint16_t _esc_status_counter{0};


        perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
        perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": update interval")};
        perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": comms errors")};

        // QGC param
        DEFINE_PARAMETERS(

            (ParamInt<px4::params::OMNI_UART_BAUD>)_param_omni_uart_baud, (ParamInt<px4::params::MOTOR_ZERO_POS>)_param_omni_motor_zero_flag,
            (ParamInt<px4::params::ENCODER_ROT_DIR>)_param_encoder_rev_flag, (ParamInt<px4::params::FRAME_ENABLE>)_param_omni_frame_enable_flag

        )
};
