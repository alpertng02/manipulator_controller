#ifndef __MANIPULATOR_PACKETS_H__
#define __MANIPULATOR_PACKETS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "manipulator_packet_types.h"


    /* ==============================
     * API
     * ============================== */

     /* --- Serialization helpers --- */
    uint16_t manipulator_next_sequence(void);
    uint32_t manipulator_crc32(const void* data, size_t len);

    /* --- Building packets --- */
    size_t manipulator_packet_header_size(void);

    size_t manipulator_packet_enable_size(void);
    size_t manipulator_packet_joint_floats_size(void);
    size_t manipulator_packet_gripper_floats_size(void);

    size_t manipulator_packet_size_for_request(void);
    size_t manipulator_packet_size_for_joint_velocities(void);
    size_t manipulator_packet_size_for_gripper_duties(void);
    size_t manipulator_packet_size_for_joint_trajectories(uint32_t point_count);
    size_t manipulator_packet_size_for_init(void);
    size_t manipulator_packet_size_for_device_state(void);
    size_t manipulator_packet_size_for_feedback(void);


    /* --- Packing --- */
    bool manipulator_pack_joint_velocities_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const float velocities[MANIPULATOR_JOINT_MOTOR_COUNT]);

    bool manipulator_pack_joint_trajectories_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        uint32_t point_count,
        const JointTrajectoryPoint* points);

    bool manipulator_pack_gripper_duties_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const float duties[MANIPULATOR_GRIPPER_MOTOR_COUNT]);

    bool manipulator_pack_init_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const ManipulatorInitPacket* init);

    bool manipulator_pack_enable_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us, ManipulatorCommands command, bool enable);

    bool manipulator_pack_request_device_state_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us);

    bool manipulator_pack_request_feedback_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us);

    bool manipulator_pack_device_state_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const ManipulatorDeviceState* state);

    bool manipulator_pack_feedback_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const ManipulatorFeedback* feedback);

    /* --- Unpacking --- */

    bool manipulator_unpack_packet_header(const uint8_t* buf, size_t buf_size,
        ManipulatorPacketHeader* out_header);

    /* --- Unpacking Payloads --- */

    bool manipulator_unpack_joint_velocities_from_payload(const uint8_t* payload, size_t payload_size,
        float out_velocities[MANIPULATOR_JOINT_MOTOR_COUNT]);

    const JointTrajectoryPoint* manipulator_unpack_joint_trajectories_from_payload(
        const uint8_t* payload, size_t payload_size, uint32_t* out_point_count);

    bool manipulator_unpack_gripper_duties_from_payload(const uint8_t* payload, size_t payload_size,
        float out_duties[MANIPULATOR_GRIPPER_MOTOR_COUNT]);

    bool manipulator_unpack_init_packet_from_payload(const uint8_t* payload, size_t payload_size,
        ManipulatorInitPacket* out_init);

    bool manipulator_unpack_enable_from_payload(const uint8_t* payload, size_t payload_size,
        bool* out_enable);

    bool manipulator_unpack_device_state_packet_from_payload(const uint8_t* payload, size_t payload_size,
        ManipulatorDeviceState* out_state);

    bool manipulator_unpack_feedback_packet_from_payload(const uint8_t* payload, size_t payload_size,
        ManipulatorFeedback* out_feedback);

    /* --- Unpacking Packets --- */

    bool unpack_joint_velocities_from_packet(const uint8_t* buf, size_t buf_size,
        float out_velocities[MANIPULATOR_JOINT_MOTOR_COUNT],
        ManipulatorPacketHeader* out_header);

    const JointTrajectoryPoint* unpack_joint_trajectories_from_packet(const uint8_t* buf, size_t buf_size,
        uint32_t* out_point_count,
        ManipulatorPacketHeader* out_header);

    bool unpack_gripper_duties_from_packet(const uint8_t* buf, size_t buf_size,
        float out_duties[MANIPULATOR_GRIPPER_MOTOR_COUNT],
        ManipulatorPacketHeader* out_header);

    bool unpack_manipulator_init_packet(const uint8_t* buf, size_t buf_size,
        ManipulatorInitPacket* out_init,
        ManipulatorPacketHeader* out_header);



#ifdef __cplusplus
}
#endif

#endif // __MANIPULATOR_PACKETS_H__