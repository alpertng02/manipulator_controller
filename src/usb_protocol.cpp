/**
 *@file usb_protocol.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "usb_protocol.hpp"
#include <vector>

bool UsbProtocol::open(const std::string& port) {
    char errorOpening = serial_.openDevice(port.c_str(), 115200);
    if (errorOpening != 1) {
        is_open_ = false;
    } else {
        is_open_ = true;
    }
    return is_open_;
}

bool UsbProtocol::is_open() {
    return serial_.isDeviceOpen() && is_open_;
}

std::vector<std::string> UsbProtocol::list_all_devices() {
    std::vector<std::string> all_devices {};

    char device_name[32] = { 0 };
    this->close();
    // Test each port between COM1 and COM99 on Windows and between /dev/ttyS0 and /dev/ttyS99 on Linux
    for (int i = 1; i < 99; i++)
    {
        // Prepare the port name (Windows)
#if defined (_WIN32) || defined( _WIN64)
        sprintf(device_name, "\\\\.\\COM%d", i);
#endif

        // Prepare the port name (Linux)
#ifdef __linux__
        sprintf(device_name, "/dev/ttyACM%d", i - 1);
#endif

        // try to connect to the device
        if (this->open(device_name))
        {
            all_devices.push_back(device_name);
            // Close the device before testing the next port
            this->close();
        }
    }
    return all_devices;
}


int UsbProtocol::write_bytes(void* buffer, const unsigned int n_bytes) {
    unsigned num_bytes = 0;
    int ret = serial_.writeBytes(buffer, n_bytes, &num_bytes);
    if (ret == 1) {
        is_open_ = true;
        return (int) num_bytes;
    } else {
        is_open_ = false;
        return 0;
    }
}

int UsbProtocol::read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) {
    int ret = serial_.readBytes(buffer, max_length, timeout.count());
    if (ret >= 0) {
        return ret;
    } else {
        return 0;
    }
}

void UsbProtocol::close() {
    serial_.closeDevice();
    is_open_ = false;
}


UsbProtocol::~UsbProtocol() {
    close();
}
