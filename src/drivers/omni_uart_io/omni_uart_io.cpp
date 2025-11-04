/**
 * @file omni_uart_io.cpp
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

// Serial Port Mapping see <https://docs.px4.io/main/en/flight_controller/pixhawk4>
// UART	Device	Port
// UART1	/dev/ttyS0	GPS
// USART2	/dev/ttyS1	TELEM1 (flow control)
// USART3	/dev/ttyS2	TELEM2 (flow control)
// UART4	/dev/ttyS3	TELEM4
// USART6	/dev/ttyS4	RC SBUS
// UART7	/dev/ttyS5	Debug Console
// UART8	/dev/ttyS6	PX4IO

#include "omni_uart_io.hpp"
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include "px4_platform_common/defines.h"

constexpr float DEG_2_RAD = static_cast<float>(M_PI) / 180.0f;
constexpr float RAD_2_DEG = 180.0f / static_cast<float>(M_PI);

OmniSerialInterface::OmniSerialInterface(const char* uart_device)
    : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}

OmniSerialInterface::~OmniSerialInterface() {
    DeinitSerial();

    ScheduleClear();
}

void OmniSerialInterface::Run() {

    // Start the loop timer
    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);

    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        // update parameters from storage
        updateParams();
    }

    if (CheckForRx()) {
        ProcessSerialRx();
        // ProcessSerialTx();
    }
    ProcessSerialTx();
    perf_end(_loop_perf);
}

bool OmniSerialInterface::init(const char* uart_device) {

    InitSerial(uart_device);

    //     ScheduleNow();
    ScheduleOnInterval(5_ms);  // 50 Hz

    return true;
}

void OmniSerialInterface::DeinitSerial() {
    if (_uart_fd >= 0) {
        close(_uart_fd);
        _uart_fd = -1;
    }
}

int OmniSerialInterface::InitSerial(const char* uart_device) {

    // Make sure we're starting clean
    DeinitSerial();

    // store port name
    strncpy(_port_in_use, uart_device, sizeof(_port_in_use) - 1);

    ConfigureSerialPeripheral(_param_omni_uart_baud.get());

    return 0;
}

int OmniSerialInterface::ConfigureSerialPeripheral(unsigned baud) {
    _uart_fd = ::open(_port_in_use, O_RDWR | O_NOCTTY);
    if (_uart_fd < 0) {
        PX4_ERR("failed to open serial port, err: %d", errno);
        return -errno;
    }

    int speed;
    switch (baud) {

        case 57600:
            speed = B57600;
            break;

        case 115200:
            speed = B115200;
            break;

        case 230400:
            speed = B230400;
            break;

        case 460800:
            speed = B460800;
            break;

        case 500000:
            speed = B500000;
            break;

        case 921600:
            speed = B921600;
            break;

        default:
            return -EINVAL;
    }

    struct termios uart_config {};
    if (tcgetattr(_uart_fd, &uart_config) != 0) {
        PX4_ERR("tcgetattr failed (%d)", errno);
        return -1;
    }

    // 原始模式，不进行换行、回车或流控处理
    cfmakeraw(&uart_config);

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;  // 将NL转换成CR(回车)-NL后输出。

    /* 无偶校验，一个停止位 */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);  // CSTOPB 使用两个停止位，PARENB 表示偶校验

    // 设置波特率
    cfsetispeed(&uart_config, speed);
    cfsetospeed(&uart_config, speed);

    // 非阻塞读取
    uart_config.c_cc[VMIN] = 0;
    uart_config.c_cc[VTIME] = 1;

    if (tcsetattr(_uart_fd, TCSANOW, &uart_config) != 0) {
        PX4_ERR("tcsetattr failed (%d)", errno);
        return -1;
    }

    close(_uart_fd);
    _uart_fd = -1;

    PX4_INFO("Serial port %s configured at %u baud", _port_in_use, baud);
    return 0;
}

bool OmniSerialInterface::CheckForRx() {

    ReOpenSerial();

    // read from the uart. This must be non-blocking, so check first if there is data available
    _bytes_available = 0;
    int ret = ioctl(_uart_fd, FIONREAD, &_bytes_available);

    if (ret != 0) {
        PX4_ERR("Reading error");
        return -1;
    }

    //     PX4_INFO("CheckForRx: available=%d bytes", (int)_bytes_available);

    return _bytes_available > 0;
}

void OmniSerialInterface::ProcessSerialRx() {

    // === 1. 从串口读取数据 ===
    ssize_t bytes_read = ::read(_uart_fd, _rx_buf, _bytes_available);

    if (bytes_read < FRAME_LEN_RX) {
        // perf_count(_comms_errors);
        return;
    };

    // === 2. 查找起始标志 (0xABCD) ===
    int start_index = -1;
    for (ssize_t i = 0; i < bytes_read - 1; i++) {
        if (_rx_buf[i] == FRAME_HEADER_1 && _rx_buf[i + 1] == FRAME_HEADER_2) {
            start_index = i;
            break;
        }
    }

    if (start_index < 0) {
        // perf_count(_comms_errors);
        return;
    }

    // === 3. 校验长度是否足够 ===
    if (bytes_read - start_index < FRAME_LEN_RX) {
        // perf_count(_comms_errors);
        return;
    }

    const uint8_t* frame = &_rx_buf[start_index];

    // === 4. 校验帧尾是否正确 (0x0D 0x0A) ===
    if (frame[FRAME_LEN_RX - 2] != FRAME_END_1 || frame[FRAME_LEN_RX - 1] != FRAME_END_2) {
        // PX4_WARN("Invalid frame tail: 0x%02X 0x%02X", frame[FRAME_LEN_RX - 2], frame[FRAME_LEN_RX - 1]);
        // perf_count(_comms_errors);
        return;
    }

    // === 5. 校验和验证 ===
    uint8_t check_sum = calcChecksum(&frame[0], FRAME_LEN_RX - 3);
    uint8_t recv_sum = frame[FRAME_LEN_RX - 3];

    if (check_sum != recv_sum) {
        // PX4_WARN("Checksum mismatch calc=0x%02X recv=0x%02X", check_sum, recv_sum);
        // perf_count(_comms_errors);
        return;
    }

    // === 6. 解析字段 ===
    uint8_t motor_index = frame[2];
    float motor_pos = bytesToFloat(&frame[4]);
    float motor_vel = bytesToFloat(&frame[8]);
    uint32_t ua = bytesToUint16(&frame[12]);
    uint32_t us = bytesToUint16(&frame[14]);
    uint32_t u = bytesToUint16(&frame[16]);

    _motor_telemetry.index = motor_index;
    _motor_telemetry.obs_angle_deg = motor_pos * RAD_2_DEG;
    _motor_telemetry.obs_angle_rad = motor_pos;
    _motor_telemetry.obs_rpm = motor_vel;
    _motor_telemetry.throttle_ua = (float)ua;
    _motor_telemetry.throttle_us = (float)us;
    _motor_telemetry.throttle_u = (float)u;
    _motor_telemetry.timestamp = hrt_absolute_time();

    _motor_telemetry_pub.publish(_motor_telemetry);

    // === 打印完整帧数据（十六进制） ===
    //     PX4_INFO_RAW(" [ProcessSerialRx]: Received From Motor frame (%d bytes): ", FRAME_LEN_RX);
    //     for (int i = 0; i < FRAME_LEN_RX; i++) {
    //         PX4_INFO_RAW("%02X ", frame[i]);
    //     }
    //     PX4_INFO_RAW("\n");
    //     // === 打印调试信息 ===
    //     PX4_INFO("[ProcessSerialRx]: Received From Motor: idx=%d, rad=%.3f, deg=%.3f, rpm=%.3f, ua=%.3f, us=%.3f, u=%.3f, t=%llu",
    //     _motor_telemetry.index,
    //              (double)_motor_telemetry.obs_angle_rad, (double)_motor_telemetry.obs_angle_deg, (double)_motor_telemetry.obs_rpm,
    //              (double)_motor_telemetry.throttle_ua, (double)_motor_telemetry.throttle_us, (double)_motor_telemetry.throttle_u,
    //              (unsigned long long)_motor_telemetry.timestamp);

    return;
}

float OmniSerialInterface::bytesToFloat(const uint8_t* bytes) {
    uint8_t b[4] = {bytes[3], bytes[2], bytes[1], bytes[0]};
    float val;
    std::memcpy(&val, b, sizeof(float));
    return val;
}

uint32_t OmniSerialInterface::bytesToUint16(const uint8_t* bytes) {
    return (uint32_t(bytes[0]) << 8) | (uint32_t(bytes[1]));
}

void OmniSerialInterface::ProcessSerialTx() {

    ReOpenSerial();

    if (_param_omni_frame_enable_flag.get() == PX4_OK) {
        setMotorZeroPosAndRev();
        return;
    } else {
        if (_single_output_cmd_sub.update(&_single_output_cmd)) {
            uint8_t frame_[FRAME_LEN_TX];
            uint8_t frame_len_ = 0;
            packThrottleCmd(_single_output_cmd, frame_, frame_len_);
            int ret = 0;

            ret = ::write(_uart_fd, frame_, frame_len_);

            if (ret != frame_len_) {
                perf_count(_comms_errors);
                PX4_ERR("UART write ret=%d, errno=%d, fd=%d", ret, errno, _uart_fd);
                // Flush data written, not transmitted
                tcflush(_uart_fd, TCOFLUSH);
            }
        }
    }
}

uint8_t OmniSerialInterface::calcChecksum(const uint8_t* data, size_t len) {
    uint32_t sum = 0;

    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }

    return static_cast<uint8_t>(sum & 0xFF);
}

void OmniSerialInterface::packThrottleCmd(const omni_outputs_cmd_s& packet, uint8_t* frame, uint8_t& frame_len) {

    // frame header
    frame[frame_len++] = FRAME_HEADER_1;
    frame[frame_len++] = FRAME_HEADER_2;

    // motor index
    frame[frame_len++] = packet.index;

    // === 启动标志位 ===
    frame[frame_len++] = _param_omni_frame_enable_flag.get();

    // === 零位校正标志位 ===
    frame[frame_len++] = _param_omni_motor_zero_flag.get();

    // === 编码器角度反转标志位 ===
    frame[frame_len++] = _param_encoder_rev_flag.get();

    // throttle ua
    uint16_t ua_val = static_cast<uint32_t>(packet.throttle_ua);
    frame[frame_len++] = (ua_val >> 8) & 0xFF;
    frame[frame_len++] = ua_val & 0xFF;

    uint16_t us_val = static_cast<uint32_t>(packet.throttle_us);  // ← 注意这里是 throttle_us
    frame[frame_len++] = (us_val >> 8) & 0xFF;
    frame[frame_len++] = us_val & 0xFF;

    // throttle us
    uint8_t phase_bytes[4];
    memcpy(phase_bytes, &packet.throttle_ctrls_phase, sizeof(float));
    frame[frame_len++] = phase_bytes[3];
    frame[frame_len++] = phase_bytes[2];
    frame[frame_len++] = phase_bytes[1];
    frame[frame_len++] = phase_bytes[0];

    // === 相位滞后角度 (float32, 高位在前) ===
    uint8_t lag_angle_bytes[4];
    memcpy(lag_angle_bytes, &packet.throttle_ctrls_lag_angle, sizeof(float));
    frame[frame_len++] = lag_angle_bytes[3];
    frame[frame_len++] = lag_angle_bytes[2];
    frame[frame_len++] = lag_angle_bytes[1];
    frame[frame_len++] = lag_angle_bytes[0];

    uint8_t check_sum = calcChecksum(&frame[0], frame_len);
    frame[frame_len++] = check_sum;

    // === 帧尾 ===
    frame[frame_len++] = FRAME_END_1;  // 0x0D
    frame[frame_len++] = FRAME_END_2;  // 0x0A

    //     === 打印调试信息 ===
    //     PX4_INFO_RAW("[ProcessSerialTx]: Send 2 Motor Frame (%zu bytes): ", frame_len);
    //     for (size_t i = 0; i < frame_len; i++) {
    //         PX4_INFO_RAW("%02X ", frame[i]);
    //     }
    //     PX4_INFO_RAW("\n");

    omni_outputs_cmd_frame_s omni_outputs_cmd_frame{};
    omni_outputs_cmd_frame.index = packet.index;
    omni_outputs_cmd_frame.throttle_ua = packet.throttle_ua;
    omni_outputs_cmd_frame.throttle_us = packet.throttle_us;
    omni_outputs_cmd_frame.throttle_ctrls_phase = packet.throttle_ctrls_phase;
    omni_outputs_cmd_frame.throttle_ctrls_lag_angle = packet.throttle_ctrls_lag_angle;
    omni_outputs_cmd_frame.frame_enable_flag = _param_omni_frame_enable_flag.get();
    omni_outputs_cmd_frame.motor_zero_set_flag = _param_omni_motor_zero_flag.get();
    omni_outputs_cmd_frame.encoder_reverse_flag = _param_encoder_rev_flag.get();
    omni_outputs_cmd_frame.timestamp = hrt_absolute_time();
    _omni_outputs_cmd_frame_pub.publish(omni_outputs_cmd_frame);
}

void OmniSerialInterface::print_info() {
    perf_print_counter(_loop_perf);
    perf_print_counter(_loop_interval_perf);
}

void OmniSerialInterface::ReOpenSerial() {

    if (_uart_fd < 0) {
        _uart_fd = open(_port_in_use, O_RDWR | O_NOCTTY | O_NONBLOCK);
    }
}

void OmniSerialInterface::setMotorZeroPosAndRev() {
    omni_outputs_cmd_s pack_cmd{0};

    uint8_t frame_[FRAME_LEN_TX];
    uint8_t frame_len_ = 0;
    packThrottleCmd(pack_cmd, frame_, frame_len_);
    int ret = 0;

    ret = ::write(_uart_fd, frame_, frame_len_);

    if (ret != frame_len_) {
        perf_count(_comms_errors);
        PX4_ERR("UART write ret=%d, errno=%d, fd=%d", ret, errno, _uart_fd);
        // Flush data written, not transmitted
        tcflush(_uart_fd, TCOFLUSH);
    }
}

/**
 * Local functions in support of the shell command.
 */
namespace omni_uart_namespace {

OmniSerialInterface* g_dev{nullptr};

int start(const char* port);
int status();
int stop();
int usage();

int start(const char* port) {
    if (g_dev != nullptr) {
        PX4_ERR("already started");
        return PX4_OK;
    }

    // Instantiate the driver.
    g_dev = new OmniSerialInterface(port);

    if (g_dev == nullptr) {
        PX4_ERR("driver start failed");
        return PX4_ERROR;
    }

    if (!g_dev->init(port)) {
        PX4_ERR("driver start failed");
        delete g_dev;
        g_dev = nullptr;
        return PX4_ERROR;
    }

    return PX4_OK;
}

int status() {
    if (g_dev == nullptr) {
        PX4_ERR("driver not running");
        return 1;
    }

    printf("state @ %p\n", g_dev);
    g_dev->print_info();

    return 0;
}

int stop() {
    if (g_dev != nullptr) {
        PX4_INFO("stopping driver");
        delete g_dev;
        g_dev = nullptr;
        PX4_INFO("driver stopped");
    } else {
        PX4_ERR("driver not running");
        return 1;
    }

    return PX4_OK;
}

int usage() {
    PRINT_MODULE_USAGE_NAME("omni_uart_interface", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_ARG("<device>", "UART device", false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
    return PX4_OK;
}

}  // namespace omni_uart_namespace
extern "C" __EXPORT int omni_uart_io_main(int argc, char* argv[]) {
    if (argc < 2) {
        PX4_WARN("Usage: omni_uart_interface {start|stop|status}");
        return PX4_ERROR;
    }

    const char* device_path = "/dev/ttyS3";

    if (!strcmp(argv[1], "start")) {
        PX4_INFO("Starting omni_uart_interface on %s", device_path);
        return omni_uart_namespace::start(device_path);

    } else if (!strcmp(argv[1], "stop")) {
        return omni_uart_namespace::stop();

    } else if (!strcmp(argv[1], "status")) {
        return omni_uart_namespace::status();
    }

    PX4_WARN("Unrecognized command: %s", argv[1]);
    return PX4_ERROR;
}
