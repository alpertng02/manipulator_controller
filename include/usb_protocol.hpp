#ifndef __USB_PROTOCOL_H__
#define __USB_PROTOCOL_H__

#include "icommunication_protocol.hpp"
#include "manipulator_packets.h"
#include "serialib.h"

class UsbProtocol : public ICommunicationProtocol {
public:
    UsbProtocol() = default;

    bool open(const std::string& port) override;

    std::vector<std::string> list_all_devices() override;

    bool is_open() override;

    int write_bytes(void* buffer, const unsigned int n_bytes) override;

    int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) override;

    void close() override;

    ~UsbProtocol() override;

private:
    serialib serial_ {};
    char read_buffer_[1024] {};
    bool is_open_ { false };
};



#endif // __USB_PROTOCOL_H__