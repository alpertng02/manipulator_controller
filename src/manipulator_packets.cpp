#include "manipulator_packets.hpp"

#include <stdexcept>
#include <cstring>

#include "manipulator_packets.h"

std::vector<uint8_t> manipulator::packets::
pack_joint_velocities(
    const std::array<float, k_joint_motor_count>& velocities,
    uint64_t time_since_boot_us) {
    if (velocities.size() != k_joint_motor_count) {
        throw std::invalid_argument("pack_joint_velocities: wrong number of velocities");
    }

    std::vector<uint8_t> buffer(
        manipulator_packet_size_for_joint_velocities(), 0);

    if (!manipulator_pack_joint_velocities_packet(buffer.data(), buffer.size(),
        time_since_boot_us,
        velocities.data()))
    {
        throw std::runtime_error("pack_joint_velocities: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> manipulator::packets::
pack_joint_trajectories(
    const std::vector<JointTrajectoryPoint>& points,
    uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        manipulator_packet_size_for_joint_trajectories(points.size()), 0);

    if (!manipulator_pack_joint_trajectories_packet(buffer.data(), buffer.size(),
        time_since_boot_us,
        points.size(),
        points.data()))
    {
        throw std::runtime_error("pack_joint_trajectories: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> manipulator::packets::
pack_gripper_duties(
    const std::array<float, k_gripper_motor_count>& duties,
    uint64_t time_since_boot_us) {
    if (duties.size() != k_gripper_motor_count) {
        throw std::invalid_argument("pack_gripper_duties: wrong number of duties");
    }

    std::vector<uint8_t> buffer(
        manipulator_packet_size_for_gripper_duties(), 0);

    if (!manipulator_pack_gripper_duties_packet(buffer.data(), buffer.size(),
        time_since_boot_us,
        duties.data()))
    {
        throw std::runtime_error("pack_gripper_duties: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> manipulator::packets::
pack_joint_feedback(const Feedback& feedback, uint64_t time_since_boot_us) {

    std::vector<uint8_t> buffer(
        manipulator_packet_size_for_feedback(), 0);

    if (!manipulator_pack_feedback_packet(buffer.data(), buffer.size(),
        time_since_boot_us, &feedback))
    {
        throw std::runtime_error("pack_joint_feedback: packet packing failed");
    }

    return buffer;
}


std::vector<uint8_t> manipulator::packets::pack_device_state(const DeviceState& state, uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        manipulator_packet_size_for_device_state(), 0);

    if (!manipulator_pack_device_state_packet(buffer.data(), buffer.size(),
        time_since_boot_us, &state))
    {
        throw std::runtime_error("pack_device_state: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> manipulator::packets::pack_running_mode_enable(const bool enable, uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        manipulator_packet_enable_size(), 0);

    if (!manipulator_pack_enable_packet(buffer.data(), buffer.size(),
        time_since_boot_us, manipulator::packets::Commands::running_mode_enable, enable))
    {
        throw std::runtime_error("pack_running_mode_enable: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> manipulator::packets::pack_init_mode_enable(const bool enable, uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        manipulator_packet_enable_size(), 0);

    if (!manipulator_pack_enable_packet(buffer.data(), buffer.size(),
        time_since_boot_us, manipulator::packets::Commands::init_mode_enable, enable))
    {
        throw std::runtime_error("pack_device_state: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> manipulator::packets::pack_request_device_state(uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        manipulator_packet_size_for_request(), 0);

    if (!manipulator_pack_request_device_state_packet(buffer.data(), buffer.size(),
        time_since_boot_us))
    {
        throw std::runtime_error("pack_request_device_state: packet packing failed");
    }

    return buffer;
}

std::vector<uint8_t> manipulator::packets::pack_request_feedback(uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        manipulator_packet_size_for_request(), 0);

    if (!manipulator_pack_request_feedback_packet(buffer.data(), buffer.size(),
        time_since_boot_us))
    {
        throw std::runtime_error("pack_request_feedback: packet packing failed");
    }

    return buffer;
}


std::vector<uint8_t> manipulator::packets::
pack_init_packet(const InitPacket& init, uint64_t time_since_boot_us) {
    std::vector<uint8_t> buffer(
        manipulator_packet_size_for_init(), 0);

    if (!manipulator_pack_init_packet(buffer.data(), buffer.size(),
        time_since_boot_us, &init))
    {
        throw std::runtime_error("pack_gripper_duties: packet packing failed");
    }

    return buffer;
}


manipulator::packets::
Header manipulator::packets::
unpack_header(const std::vector<uint8_t>& buf) {
    if (buf.size() < manipulator_packet_header_size()) {
        throw std::invalid_argument("unpack_header: buffer too small");
    }

    manipulator::packets::
        Header hdr {};
    if (!manipulator_unpack_packet_header(buf.data(), buf.size(), &hdr)) {
        throw std::runtime_error("unpack_header: unpacking unsuccesful");
    }

    return hdr;
}

manipulator::packets::
Packet manipulator::packets::
unpack_packet(const std::vector<uint8_t>& packet) {
    if (packet.empty())
        throw std::invalid_argument("unpack_packet: packet is empty");

    manipulator::packets::
        Packet unpacked_packet {};

    unpacked_packet.header = unpack_header(packet);

    if (unpacked_packet.header.content_size <= 0) {
        throw std::runtime_error("unpack_packet: No content exists");
    }

    if (packet.size() < manipulator_packet_header_size() + unpacked_packet.header.content_size)
        throw std::runtime_error("unpack_packet: buffer smaller than expected content size");

    unpacked_packet.payload.assign(
        packet.begin() + manipulator_packet_header_size(),
        packet.begin() + manipulator_packet_header_size() + unpacked_packet.header.content_size);

    return unpacked_packet;
};


std::array<float, manipulator::packets::
    k_joint_motor_count> manipulator::packets::
    unpack_joint_velocities_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty())
        throw std::invalid_argument("unpack_joint_velocities_payload: payload is empty");

    std::array<float, manipulator::packets::
        k_joint_motor_count> velocities {};
    if (!manipulator_unpack_joint_velocities_from_payload(
        payload.data(), payload.size(), velocities.data()))
    {
        throw std::runtime_error("unpack_joint_velocities_payload: failed to unpack payload");
    }
    return velocities;
};

std::vector<JointTrajectoryPoint> manipulator::packets::
unpack_joint_trajectories_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty())
        throw std::invalid_argument("unpack_joint_trajectories_payload: payload is empty");

    uint32_t count = 0;
    const JointTrajectoryPoint* points =
        manipulator_unpack_joint_trajectories_from_payload(
            payload.data(), payload.size(), &count);

    if (!points)
        throw std::runtime_error("unpack_joint_trajectories_payload: failed to unpack packet");

    std::vector<JointTrajectoryPoint> trajectory_points {};
    trajectory_points.assign(points, points + count);
    return trajectory_points;
}

std::array<float, manipulator::packets::
    k_gripper_motor_count> manipulator::packets::
    unpack_gripper_duties_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty())
        throw std::invalid_argument("unpack_gripper_duties_payload: payload empty");

    std::array<float, k_gripper_motor_count> duties {};
    if (!manipulator_unpack_gripper_duties_from_payload(
        payload.data(), payload.size(), duties.data()))
    {
        throw std::runtime_error("unpack_gripper_duties_payload: failed to unpack payload");
    }
    return duties;
}

manipulator::packets::DeviceState
manipulator::packets::unpack_device_state_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty()) {
        throw std::invalid_argument("unpack_device_state_payload: payload empty");
    }

    DeviceState state {};
    if (!manipulator_unpack_device_state_packet_from_payload(payload.data(), payload.size(), &state)) {
        throw std::runtime_error("unpack_device_state_payload: failed to unpack payload");
    }
    return state;
}

manipulator::packets::Feedback
manipulator::packets::unpack_joint_feedback_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty()) {
        throw std::invalid_argument("unpack_joint_feedback_payload: payload empty");
    }

    Feedback feedback {};
    if (!manipulator_unpack_feedback_packet_from_payload(payload.data(), payload.size(), &feedback)) {
        throw std::runtime_error("unpack_joint_feedback_payload: failed to unpack payload");
    }
    return feedback;
}

manipulator::packets::InitPacket
manipulator::packets::unpack_init_packet_payload(const std::vector<uint8_t>& payload) {
    if (payload.empty()) {
        throw std::invalid_argument("unpack_init_packet_payload: payload empty");
    }

    InitPacket init {};
    if (!manipulator_unpack_init_packet_from_payload(payload.data(), payload.size(), &init)) {
        throw std::runtime_error("unpack_init_packet_payload: failed to unpack payload");
    }
    return init;
}


