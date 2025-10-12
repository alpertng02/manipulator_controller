#ifndef __USB_DEVICE_H__
#define __USB_DEVICE_H__

#include "imanipulator_device.hpp"
#include "manipulator_packets.h"
#include "serialib.h"

class UsbManipulatorDevice : public IManipulatorDevice {
public:
    UsbManipulatorDevice();

    bool open(const std::string& port) override;

    std::vector<std::string> list_all_ports() override;

    bool is_open() override;

    void close() override;

    ~UsbManipulatorDevice() override;

private:
    serialib serial_ {};
    char read_buffer_[1024] {};
    bool is_open_ { false };

    int write_bytes(void* buffer, const unsigned int n_bytes) override;

    int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) override;
};



#endif // __USB_DEVICE_H__