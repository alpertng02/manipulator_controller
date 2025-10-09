/**
 *@file icommunication_protocol.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-10-02
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "imanipulator_device.hpp"
#include "manipulator_packets.hpp"
#include "manipulator_packets.h"
#include <chrono>
#include <thread>
#include <vector>
#include <stdexcept>
#include <cstring>


IManipulatorDevice::~IManipulatorDevice() = default;

bool IManipulatorDevice::send_joint_velocities(const std::array<float, manipulator::packets::k_joint_motor_count>& velocities) {
    try {
        auto packet = manipulator::packets::pack_joint_velocities(velocities, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}

bool IManipulatorDevice::send_gripper_dutycycles(const std::array<float, manipulator::packets::k_gripper_motor_count>& dutycycles) {
    try {
        auto packet = manipulator::packets::pack_gripper_duties(dutycycles, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}

bool IManipulatorDevice::send_init_packet(const manipulator::packets::InitPacket& init_packet) {
    try {
        auto packet = manipulator::packets::pack_init_packet(init_packet, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}

bool IManipulatorDevice::send_init_mode_enable(const bool enable) {
    try {
        auto packet = manipulator::packets::pack_init_mode_enable(enable, 0);
        int bytes_sent = this->write_bytes(packet.data(), packet.size());
        if (bytes_sent > 0) {
            return static_cast<size_t>(bytes_sent) == packet.size();
        }
    } catch (std::runtime_error& err) {
        return false;
    }

    return false;
}


bool IManipulatorDevice::init_device(const manipulator::packets::InitPacket& init_packet, std::chrono::milliseconds timeout) {
    auto start_time = std::chrono::steady_clock::now();
    auto device_state = this->receive_device_state(timeout);
    while (device_state.device_is_init) {
        auto current_time = std::chrono::steady_clock::now();
        auto past_time = current_time - start_time;

        this->send_init_mode_enable(true);
        device_state = this->receive_device_state(std::chrono::duration_cast<std::chrono::milliseconds>(timeout - past_time));

        if (past_time > timeout) {
            return false;
        }
    }

    bool res = this->send_init_packet(init_packet);
    return res;
}


manipulator::packets::Feedback IManipulatorDevice::receive_joint_feedback(std::chrono::milliseconds timeout) {
    auto request_packet = manipulator::packets::pack_request_feedback(0);
    int bytes_sent = this->write_bytes(request_packet.data(), request_packet.size());
    if (bytes_sent < 0) {
        throw std::runtime_error("receive_joint_feedback: could not send request packet");
    }

    if (static_cast<size_t>(bytes_sent) != request_packet.size()) {
        throw std::runtime_error("receive_joint_feedback: could not send all bytes of packet");
    }

    auto packet = this->receive_packet(timeout);
    auto unpacked = manipulator::packets::unpack_packet(packet);

    if (unpacked.header.command_id == manipulator::packets::Commands::joint_feedback) {
        auto joint_feedback = manipulator::packets::unpack_joint_feedback_payload(unpacked.payload);
        return joint_feedback;
    } else {
        throw std::runtime_error("receive_joint_feedback: package received is not package");
    }

}

manipulator::packets::DeviceState IManipulatorDevice::receive_device_state(std::chrono::milliseconds timeout) {
    auto request_packet = manipulator::packets::pack_request_device_state(0);
    int bytes_sent = this->write_bytes(request_packet.data(), request_packet.size());
    if (bytes_sent < 0) {
        throw std::runtime_error("receive_device_state: could not send request packet");
    }

    if (static_cast<size_t>(bytes_sent) != request_packet.size()) {
        throw std::runtime_error("receive_device_state: could not send all bytes of packet");
    }

    auto packet = this->receive_packet(timeout);
    auto unpacked = manipulator::packets::unpack_packet(packet);

    if (unpacked.header.command_id == manipulator::packets::Commands::device_state) {
        auto device_state = manipulator::packets::unpack_device_state_payload(unpacked.payload);
        return device_state;
    } else {
        throw std::runtime_error("receive_device_state: package received is not package");
    }

}

std::vector<uint8_t> IManipulatorDevice::receive_packet(std::chrono::milliseconds timeout) {
    using namespace manipulator::packets;

    std::vector<uint8_t> buffer;
    buffer.reserve(512);  // allow enough space for full packets

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // Check timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > timeout) {
            throw std::runtime_error("Timeout while waiting for packet");
        }

        // Read available bytes from device
        uint8_t temp[64];
        auto remaining_time = (elapsed < timeout)
            ? std::chrono::duration_cast<std::chrono::milliseconds>(timeout - elapsed)
            : std::chrono::milliseconds(0);
        int bytes_read = this->read_bytes(temp, sizeof(temp), remaining_time);
        if (bytes_read < 0) {
            throw std::runtime_error("Error reading from device");
        } else if (bytes_read == 0) {
            // No data yet
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        buffer.insert(buffer.end(), temp, temp + bytes_read);

        // Check if we have enough for a header
        while (buffer.size() >= manipulator_packet_header_size()) {
            // Try to unpack header
            Header hdr;
            try {
                hdr = unpack_header(buffer);
            } catch (const std::exception&) {
                // If header invalid, discard first byte and continue
                buffer.erase(buffer.begin());
                continue;
            }

            // Check for correct device ID (optional)
            if (hdr.device_id != MANIPULATOR_DEVICE_ID) {
                buffer.erase(buffer.begin()); // discard first byte and resync
                continue;
            }

            size_t full_packet_size = manipulator_packet_header_size() + hdr.content_size;

            // Wait for more bytes if packet incomplete
            if (buffer.size() < full_packet_size) {
                break;
            }

            // Optional: CRC check
            uint32_t crc_calc = manipulator_crc32(buffer.data(), full_packet_size - sizeof(uint32_t));
            if (crc_calc != hdr.crc32) {
                // Bad CRC, discard first byte and resync
                buffer.erase(buffer.begin());
                continue;
            }

            // Full packet valid
            std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + full_packet_size);
            buffer.erase(buffer.begin(), buffer.begin() + full_packet_size);
            return packet;
        }

        // Optional safeguard
        if (buffer.size() > 1024) {
            buffer.clear();
            throw std::runtime_error("Received buffer too large without valid packet");
        }
    }
}
