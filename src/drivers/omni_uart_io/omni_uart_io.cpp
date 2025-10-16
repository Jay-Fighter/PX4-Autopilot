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

// Serial Port Mapping see <https://docs.px4.io/main/en/flight_controller/nxp_mr_vmu_rt1176.html#serial-port-mapping>
// UART	Device	Port
// UART1	/dev/ttyS0	Debug
// UART3	/dev/ttyS1	GPS
// UART4	/dev/ttyS2	TELEM1
// UART5	/dev/ttyS3	GPS2
// UART6	/dev/ttyS4	PX4IO
// UART8	/dev/ttyS5	TELEM2
// UART10	/dev/ttyS6	TELEM3
// UART11	/dev/ttyS7	External

#include "omni_uart_io.hpp"
#include <unistd.h>
#include "px4_platform_common/defines.h"

OmniSerialInterface::OmniSerialInterface(const char* uart_device)
    : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(uart_device)) {}

OmniSerialInterface::~OmniSerialInterface() {

    ScheduleClear();
}

void OmniSerialInterface::Run() {

    // Start the loop timer
    // Increment our loop counter
    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);

    if (CheckForRx()) {
        ReadAndSetRxBytes();
    }

    // stop our timer
    perf_end(_loop_perf);
}

bool OmniSerialInterface::init(const char* uart_device) {

    InitSerial(uart_device);

//     ScheduleNow();
    ScheduleOnInterval(20_ms); // 50 Hz

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

    _uart_fd = ::open(_port_in_use, O_RDWR | O_NOCTTY);

    if (_uart_fd < 0) {
        PX4_ERR("failed to open serial port: %s err: %d", uart_device, errno);
        return -errno;
    }

    PX4_INFO("Opened serial port successfully");

    // Now that we've opened the port, fully configure it for baud/bit num
    return ConfigureSerialPeripheral(_param_omni_uart_baud.get());
}

int OmniSerialInterface::ConfigureSerialPeripheral(unsigned baud) {

    int speed;

    switch (baud) {
        case 9600:
            speed = B9600;
            break;

        case 19200:
            speed = B19200;
            break;

        case 38400:
            speed = B38400;
            break;

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

        case 1000000:
            speed = B1000000;
            break;

        default:
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(_uart_fd, &uart_config);

    //
    // Input flags - Turn off input processing
    //
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    //
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    uart_config.c_oflag = 0;

    //
    // No line processing
    //
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    /* no parity, one stop bit, disable flow control */
    uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

    /* set baud rate */
    // Set the baud rate in the input direction (when receiving)
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        return -errno;
    }
    // Set the baud rate in the output direction (when sending)
    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        return -errno;
    }

    if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
        return -errno;
    }

    close(_uart_fd);
    _uart_fd = -1;

    return 0;
}

bool OmniSerialInterface::CheckForRx() {
    // Reopen serial
    ReOpenSerial();

    /*Copy the original byte stream*/

    // read from the uart. This must be non-blocking, so check first if there is data available
    _bytes_available = 0;
    int ret = ioctl(_uart_fd, FIONREAD, (unsigned long)&_bytes_available);

    if (ret != 0) {
        PX4_ERR("Reading error");
        return -1;
    }

    return _bytes_available > 0;
}

uint8_t* OmniSerialInterface::ReadAndSetRxBytes() {
    // Read the bytes available for us
    read(_uart_fd, _rx_buf, _bytes_available);

    // TODO 在这里进行解包
    return _rx_buf;
}

void OmniSerialInterface::ProcessSerialTx() {
    // Reopen serial
    ReOpenSerial();

    // TODO 在这里进行打包 调用write进行下发
}

void OmniSerialInterface::print_info() {}

void OmniSerialInterface::ReOpenSerial() {
    if (_uart_fd < 0) {
        _uart_fd = open(_port_in_use, O_RDWR | O_NOCTTY);
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

extern "C" __EXPORT int omni_uart_interface_main(int argc, char* argv[]) {
    int ch = 0;
    const char* device_path = nullptr;
    int myoptind = 1;
    const char* myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {

            case 'd':
                device_path = myoptarg;
                break;

            default:
                PX4_WARN("Unknown option!");
                return PX4_ERROR;
        }
    }

    if (myoptind >= argc) {
        PX4_ERR("unrecognized command");
        return omni_uart_namespace::usage();
    }

    if (!device_path) {
        PX4_ERR("Missing device");
        return PX4_ERROR;
    }

    if (!strcmp(argv[myoptind], "start")) {
        if (strcmp(device_path, "") != 0) {
            return omni_uart_namespace::start(device_path);

        } else {
            PX4_WARN("Please specify device path!");
            return omni_uart_namespace::usage();
        }

    } else if (!strcmp(argv[myoptind], "stop")) {
        return omni_uart_namespace::stop();

    } else if (!strcmp(argv[myoptind], "status")) {
        return omni_uart_namespace::status();
    }

    return omni_uart_namespace::usage();
}
