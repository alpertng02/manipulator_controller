#ifndef __MANIPULATOR_PACKETS_HPP__
#define __MANIPULATOR_PACKETS_HPP__

#include "manipulator_packet_types.h"

#include <vector>
#include <array>
#include <cstdint>

namespace manipulator {

namespace packets {

constexpr uint32_t k_device_id = MANIPULATOR_DEVICE_ID;
constexpr size_t k_joint_motor_count = MANIPULATOR_JOINT_MOTOR_COUNT;
constexpr size_t k_gripper_motor_count = MANIPULATOR_GRIPPER_MOTOR_COUNT;

using Header = ManipulatorPacketHeader;
using InitPacket = ManipulatorInitPacket;
using Commands = ManipulatorCommands;
using DeviceState = ManipulatorDeviceState;
using Feedback = ManipulatorFeedback;

struct Packet {
    Header header;
    std::vector<uint8_t> payload;
};

// Packing

std::vector<uint8_t> pack_joint_velocities(const std::array<float, k_joint_motor_count>& velocities,
    uint64_t time_since_boot_us);

std::vector<uint8_t> pack_joint_trajectories(const std::vector<JointTrajectoryPoint>& points,
    uint64_t time_since_boot_us);

std::vector<uint8_t> pack_gripper_duties(const std::array<float, k_gripper_motor_count>& duties,
    uint64_t time_since_boot_us);

std::vector<uint8_t> pack_joint_feedback(const Feedback& feedback, uint64_t time_since_boot_us);

std::vector<uint8_t> pack_device_state(const DeviceState& state, uint64_t time_since_boot_us);

std::vector<uint8_t> pack_request_device_state(uint64_t time_since_boot_us);

std::vector<uint8_t> pack_request_feedback(uint64_t time_since_boot_us);

std::vector<uint8_t> pack_init_packet(const InitPacket& init, uint64_t time_since_boot_us);

std::vector<uint8_t> pack_init_mode_enable(const bool enable, uint64_t time_since_boot_us);

// Unpacking

Header unpack_header(const std::vector<uint8_t>& buf);

Packet unpack_packet(const std::vector<uint8_t>& packet);

std::array<float, k_joint_motor_count> unpack_joint_velocities_payload(const std::vector<uint8_t>& payload);

std::vector<JointTrajectoryPoint> unpack_joint_trajectories_payload(const std::vector<uint8_t>& payload);

Feedback unpack_joint_feedback_payload(const std::vector<uint8_t>& payload);

std::array<float, k_gripper_motor_count> unpack_gripper_duties_payload(const std::vector<uint8_t>& payload);

DeviceState unpack_device_state_payload(const std::vector<uint8_t>& payload);

InitPacket unpack_init_packet_payload(const std::vector<uint8_t>& payload);

}

}


#endif // __MANIPULATOR_PACKETS_HPP__