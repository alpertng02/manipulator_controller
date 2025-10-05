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

#include "icommunication_protocol.hpp"
#include <vector>

ICommunicationProtocol::~ICommunicationProtocol() {}

bool ICommunicationProtocol::send_command_packet(ManipulatorCommandPacket& packet) {
    packet.frame_start = MANIPULATOR_FRAME_START;
    packet.frame_end = MANIPULATOR_FRAME_END;
    int ret = write_bytes((void*) &packet, sizeof(packet));
    //RCLCPP_INFO(this->get_logger(), "Sent bytes: %d/%zu", ret, sizeof(packet));
    if (ret == sizeof(packet)) {
        return true;
    } else {
        return false;
    }
}

bool ICommunicationProtocol::send_init_packet(ManipulatorInitPacket& packet) {
    packet.frame_start = MANIPULATOR_FRAME_START;
    packet.frame_end = MANIPULATOR_FRAME_END;
    int ret = write_bytes((void*) &packet, sizeof(packet));
    //RCLCPP_INFO(this->get_logger(), "Sent bytes: %d/%zu", ret, sizeof(packet));
    if (ret == sizeof(packet)) {
        return true;
    } else {
        return false;
    }
}

static std::vector<uint8_t> buffer {};
bool ICommunicationProtocol::receive_motor_feedback(ManipulatorFeedbackPacket* feedback, std::chrono::milliseconds timeout) {
    if (!feedback) return false;

    uint8_t temp[64]; // read in small chunks
    int bytes_read = read_bytes(temp, sizeof(temp), timeout);

    if (bytes_read <= 0) return false;

    // Append new data to sliding buffer
    buffer.insert(buffer.end(), temp, temp + bytes_read);

    while (buffer.size() >= sizeof(ManipulatorFeedbackPacket)) {
        // Check for MANIPULATOR_FRAME_START at current position
        uint32_t frame_start = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
        if (frame_start == MANIPULATOR_FRAME_START) {
            // Possible start of packet, check if we have enough bytes
            if (buffer.size() < sizeof(ManipulatorFeedbackPacket)) break;

            ManipulatorFeedbackPacket* pkt = reinterpret_cast<ManipulatorFeedbackPacket*>(buffer.data());
            if (pkt->frame_end == MANIPULATOR_FRAME_END) {
                // Valid packet, copy to feedback
                *feedback = *pkt;
                // Remove consumed bytes
                buffer.erase(buffer.begin(), buffer.begin() + sizeof(ManipulatorFeedbackPacket));
                return true;
            } else {
                // MANIPULATOR_FRAME_END mismatch, discard first byte and resync
                buffer.erase(buffer.begin());
            }
        } else {
            // MANIPULATOR_FRAME_START not found, discard first byte
            buffer.erase(buffer.begin());
        }
    }
    return false;
}

bool ICommunicationProtocol::receive_state_feedback(ManipulatorStatePacket* packet, std::chrono::milliseconds timeout) {
    if (!packet) return false;

    uint8_t temp[64]; // read in small chunks
    int bytes_read = read_bytes(temp, sizeof(temp), timeout);

    if (bytes_read <= 0) return false;

    // Append new data to sliding buffer
    buffer.insert(buffer.end(), temp, temp + bytes_read);

    while (buffer.size() >= sizeof(ManipulatorStatePacket)) {
        // Check for MANIPULATOR_FRAME_START at current position
        uint32_t frame_start = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
        if (frame_start == MANIPULATOR_FRAME_START) {
            // Possible start of packet, check if we have enough bytes
            if (buffer.size() < sizeof(ManipulatorStatePacket)) break;

            ManipulatorStatePacket* pkt = reinterpret_cast<ManipulatorStatePacket*>(buffer.data());
            if (pkt->frame_end == MANIPULATOR_FRAME_END) {
                // Valid packet, copy to feedback
                *packet = *pkt;
                // Remove consumed bytes
                buffer.erase(buffer.begin(), buffer.begin() + sizeof(ManipulatorStatePacket));
                return true;
            } else {
                // MANIPULATOR_FRAME_END mismatch, discard first byte and resync
                buffer.erase(buffer.begin());
            }
        } else {
            // MANIPULATOR_FRAME_START not found, discard first byte
            buffer.erase(buffer.begin());
        }
    }
    return false;
}


bool ICommunicationProtocol::set_joint_velocities(const std::array<float, 3>& velocities) {
    ManipulatorCommandPacket pkt {};

    pkt.command_id = ManipulatorCommands::set_joint_velocity;
    for (size_t i = 0; i < velocities.size(); i++) {
        pkt.content.joint_velocity.velocities[i] = velocities[i];
    }
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

bool ICommunicationProtocol::set_joint_trajectories(
    const std::array<int32_t, 3>& target_positions,
    const std::array<float, 3>& maximum_velocities,
    const std::array<float, 3>& final_velocities,
    const std::array<float, 3>& accelerations,
    const std::array<float, 3>& deaccelerations,
    const std::array<float, 3>& acc_jerks,
    const std::array<float, 3>& dec_jerks) {
    
    ManipulatorCommandPacket pkt {};

    pkt.command_id = ManipulatorCommands::set_joint_trajectory;
    for (size_t i = 0; i < target_positions.size(); i++) {
        pkt.content.joint_trajectory.target_positions[i] = target_positions[i];
        pkt.content.joint_trajectory.maximum_velocities[i] = maximum_velocities[i];
        pkt.content.joint_trajectory.final_velocities[i] = final_velocities[i];
        pkt.content.joint_trajectory.accelerations[i] = accelerations[i];
        pkt.content.joint_trajectory.deaccelerations[i] = deaccelerations[i];
        pkt.content.joint_trajectory.acc_jerks[i] = acc_jerks[i];
        pkt.content.joint_trajectory.dec_jerks[i] = dec_jerks[i];
    }
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

bool ICommunicationProtocol::set_gripper_dutycycles(const std::array<float, 3>& dutycycles) {
    ManipulatorCommandPacket pkt {};

    pkt.command_id = ManipulatorCommands::set_gripper_duties;
    for (size_t i = 0; i < dutycycles.size(); i++) {
        pkt.content.gripper_duty.gripper_duties[i] = dutycycles[i];
    }
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

bool ICommunicationProtocol::set_joint_current_positions(const std::array<int32_t, 3>& current_positions) {
    ManipulatorCommandPacket pkt {};

    pkt.command_id = ManipulatorCommands::set_joint_current_pos;
    for (size_t i = 0; i < current_positions.size(); i++) {
        pkt.content.joint_current_pos.current_pos[i] = current_positions[i];
    }
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}


// Start / Stop motor control
bool ICommunicationProtocol::set_running_mode_enabled(bool enable) {
    ManipulatorCommandPacket pkt {};
    pkt.command_id = ManipulatorCommands::running_mode_enable;
    pkt.content.mode_enable.enable = static_cast<uint8_t>(enable);
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Enable / disable init mode
bool ICommunicationProtocol::set_init_mode_enabled(bool enable) {
    ManipulatorCommandPacket pkt {};
    pkt.command_id = ManipulatorCommands::init_mode_enable;
    pkt.content.mode_enable.enable = static_cast<uint8_t>(enable);
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}

// Enable / disable init mode
bool ICommunicationProtocol::request_device_state() {
    ManipulatorCommandPacket pkt {};
    pkt.command_id = ManipulatorCommands::send_device_state;
    if (!send_command_packet(pkt)) {
        return false;
    } else {
        return true;
    }
}