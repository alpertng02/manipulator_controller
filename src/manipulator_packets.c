#include "manipulator_packets.h"
#include <string.h>

#ifndef __ORDER_LITTLE_ENDIAN__
#include <endian.h>  /* for htole32, le32toh etc. */
#else 

static inline uint8_t htole8(uint8_t a) {
    return a;
}

static inline uint16_t htole16(uint16_t a) {
    return a;
}

static inline uint32_t htole32(uint32_t a) {
    return a;
}

static inline uint64_t htole64(uint64_t a) {
    return a;
}

static inline uint8_t le8toh(uint8_t a) {
    return a;
}

static inline uint16_t le16toh(uint16_t a) {
    return a;
}

static inline uint32_t le32toh(uint32_t a) {
    return a;
}

static inline uint64_t le64toh(uint64_t a) {
    return a;
}

#endif

/* ==============================
 * CRC32 Implementation (Polynomial 0xEDB88320)
 * ============================== */
uint32_t manipulator_crc32(const void* data, size_t len) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (size_t j = 0; j < 8; j++)
                c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        init = true;
    }
    uint32_t crc = 0xFFFFFFFFU;
    const uint8_t* p = (const uint8_t*) data;
    for (size_t i = 0; i < len; i++)
        crc = table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFU;
}

/* ==============================
 * Sequence Generator
 * ============================== */
uint16_t manipulator_next_sequence(void) {
    static uint16_t seq = 0;
    return ++seq;
}

/* ==============================
 * Helpers
 * ============================== */
size_t manipulator_packet_header_size(void) {
    return sizeof(ManipulatorPacketHeader);
}

size_t manipulator_packet_joint_floats_size(void) {
    return manipulator_packet_header_size() + sizeof(float) * MANIPULATOR_JOINT_MOTOR_COUNT;
}

size_t manipulator_packet_gripper_floats_size(void) {
    return manipulator_packet_header_size() + sizeof(float) * MANIPULATOR_GRIPPER_MOTOR_COUNT;
}

size_t manipulator_packet_enable_size(void) {
    return manipulator_packet_header_size() + sizeof(uint8_t);
}


size_t manipulator_packet_size_for_joint_velocities(void) {
    return manipulator_packet_joint_floats_size();
}

size_t manipulator_packet_size_for_gripper_duties(void) {
    return manipulator_packet_gripper_floats_size();
}

size_t manipulator_packet_size_for_joint_trajectories(uint32_t point_count) {
    return manipulator_packet_header_size() +
        sizeof(uint32_t) + point_count * sizeof(JointTrajectoryPoint);
}

size_t manipulator_packet_size_for_request(void) {
    return manipulator_packet_header_size() + sizeof(uint32_t);
}

size_t manipulator_packet_size_for_init(void) {
    return manipulator_packet_header_size() +
        sizeof(ManipulatorInitPacket);
}

size_t manipulator_packet_size_for_device_state(void) {
    return manipulator_packet_header_size() +
        sizeof(ManipulatorDeviceState);
}

size_t manipulator_packet_size_for_feedback(void) {
    return manipulator_packet_header_size() +
        sizeof(ManipulatorFeedback);
}


/* ==============================
 * Build Functions
 * ============================== */
bool manipulator_pack_joint_velocities_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const float velocities[MANIPULATOR_JOINT_MOTOR_COUNT]) {
    if (!buf || !velocities) return false;
    size_t payload_size = sizeof(float) * MANIPULATOR_JOINT_MOTOR_COUNT;
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(set_joint_velocities);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), velocities, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool manipulator_pack_joint_trajectories_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    uint32_t point_count,
    const JointTrajectoryPoint* points) {
    if (!buf || (point_count && !points)) return false;

    uint32_t point_count_le = htole32(point_count);
    size_t payload_size = sizeof(uint32_t) + point_count * sizeof(JointTrajectoryPoint);
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(set_joint_trajectories);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &point_count_le, sizeof(uint32_t));
    memcpy(buf + sizeof(hdr) + sizeof(uint32_t),
        points, point_count * sizeof(JointTrajectoryPoint));

    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );
    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool manipulator_pack_gripper_duties_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const float duties[MANIPULATOR_GRIPPER_MOTOR_COUNT]) {
    if (!buf || !duties) return false;
    size_t payload_size = sizeof(float) * MANIPULATOR_GRIPPER_MOTOR_COUNT;
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(set_gripper_duties);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), duties, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool manipulator_pack_request_device_state_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us) {
    if (!buf) return false;
    size_t payload_size = sizeof(uint32_t);
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(request_device_state);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    uint32_t payload_data = 0x69;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &payload_data, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool manipulator_pack_request_feedback_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us) {
    if (!buf) return false;
    size_t payload_size = sizeof(uint32_t);
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(request_joint_feedback);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    uint32_t payload_data = 0x69;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &payload_data, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool manipulator_pack_init_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const ManipulatorInitPacket* init) {
    if (!buf || !init) return false;
    size_t payload_size = sizeof(ManipulatorInitPacket);
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(init_device);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), init, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool manipulator_pack_enable_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us, ManipulatorCommands command, bool enable) {
    if (!buf) return false;
    size_t payload_size = sizeof(uint8_t);
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(command);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    uint8_t data = (enable);
    memcpy(buf + sizeof(hdr), &data, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool manipulator_pack_device_state_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const ManipulatorDeviceState* state) {
    if (!buf || !state) return false;
    size_t payload_size = sizeof(ManipulatorDeviceState);
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(device_state);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), state, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool manipulator_pack_feedback_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const ManipulatorFeedback* feedback) {
    if (!buf || !feedback) return false;
    size_t payload_size = sizeof(ManipulatorFeedback);
    size_t total_size = manipulator_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    ManipulatorPacketHeader hdr;
    hdr.device_id = htole32(MANIPULATOR_DEVICE_ID);
    hdr.command_id = htole16(joint_feedback);
    hdr.sequence = htole16(manipulator_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), feedback, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        manipulator_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

/* ==============================
 * Unpacking
 * ============================== */
bool manipulator_unpack_packet_header(const uint8_t* buf, size_t buf_size, ManipulatorPacketHeader* out_header) {
    if (!buf || buf_size < manipulator_packet_header_size())
        return false;

    ManipulatorPacketHeader hdr;
    memcpy(&hdr, buf, sizeof(hdr));

    uint32_t crc_recv = le32toh(hdr.crc32);
    uint16_t content_size = le16toh(hdr.content_size);

    size_t expected_total_size = manipulator_packet_header_size() + content_size;

    // Zero out the CRC field for verification
    ManipulatorPacketHeader temp_hdr = hdr;
    temp_hdr.crc32 = 0;

    uint8_t temp_buf[sizeof(ManipulatorPacketHeader)] = { 0 };
    memcpy((void*) temp_buf, &temp_hdr, sizeof(temp_hdr)); 

    if (buf_size > expected_total_size) {

        uint32_t crc_calc = manipulator_crc32(temp_buf, expected_total_size - sizeof(hdr.crc32));
        if (crc_calc != crc_recv)
            return false; // CRC mismatch
    }

    if (out_header) {
        hdr.device_id = le32toh(hdr.device_id);
        hdr.command_id = le16toh(hdr.command_id);
        hdr.sequence = le16toh(hdr.sequence);
        hdr.content_size = content_size;
        hdr.time_since_boot_us = le64toh(hdr.time_since_boot_us);
        hdr.crc32 = crc_recv;
        *out_header = hdr;
    }

    return true;
}


bool manipulator_unpack_joint_velocities_from_payload(const uint8_t* payload, size_t payload_size,
    float out_velocities[MANIPULATOR_JOINT_MOTOR_COUNT]) {
    if (!payload || !out_velocities) return false;
    if (payload_size != sizeof(float) * MANIPULATOR_JOINT_MOTOR_COUNT) return false;

    memcpy(out_velocities, payload,
        sizeof(float) * MANIPULATOR_JOINT_MOTOR_COUNT);

    return true;
}

const JointTrajectoryPoint* manipulator_unpack_joint_trajectories_from_payload(
    const uint8_t* payload, size_t payload_size, uint32_t* out_point_count) {

    if (!payload || !out_point_count) return NULL;

    uint32_t point_count;
    memcpy(&point_count, payload, sizeof(uint32_t));
    point_count = le32toh(point_count);

    size_t expected = sizeof(uint32_t) + point_count * sizeof(JointTrajectoryPoint);
    if (payload_size != expected) return NULL;

    *out_point_count = point_count;
    return (const JointTrajectoryPoint*) (payload + sizeof(uint32_t));
}

bool manipulator_unpack_gripper_duties_from_payload(const uint8_t* payload, size_t payload_size,
    float out_duties[MANIPULATOR_GRIPPER_MOTOR_COUNT]) {
    if (!payload || !out_duties) return false;
    if (payload_size != sizeof(float) * MANIPULATOR_GRIPPER_MOTOR_COUNT) return false;

    memcpy(out_duties, payload,
        sizeof(float) * MANIPULATOR_GRIPPER_MOTOR_COUNT);

    return true;
}

bool manipulator_unpack_init_packet_from_payload(const uint8_t* payload, size_t payload_size,
    ManipulatorInitPacket* out_init) {
    if (!payload || !out_init) return false;
    if (payload_size != sizeof(ManipulatorInitPacket)) return false;

    memcpy(out_init, payload,
        sizeof(ManipulatorInitPacket));

    return true;
}

bool manipulator_unpack_enable_from_payload(const uint8_t* payload, size_t payload_size,
    bool* out_enable) {
    if (!payload || !out_enable) return false;
    if (payload_size != sizeof(uint8_t)) return false;

    uint8_t data = 0;
    memcpy(&data, payload,
        sizeof(uint8_t));

    *out_enable = (bool) data;
    return true;
}

bool manipulator_unpack_device_state_packet_from_payload(const uint8_t* payload, size_t payload_size,
    ManipulatorDeviceState* out_state) {
    if (!payload || !out_state) return false;
    if (payload_size != sizeof(ManipulatorDeviceState)) return false;

    memcpy(out_state, payload,
        sizeof(ManipulatorDeviceState));

    return true;
}

bool manipulator_unpack_feedback_packet_from_payload(const uint8_t* payload, size_t payload_size,
    ManipulatorFeedback* out_feedback) {
    if (!payload || !out_feedback) return false;
    if (payload_size != sizeof(ManipulatorFeedback)) return false;

    memcpy(out_feedback, payload,
        sizeof(ManipulatorFeedback));

    return true;
}

bool unpack_joint_velocities_from_packet(
    const uint8_t* buf, size_t buf_size,
    float out_velocities[MANIPULATOR_JOINT_MOTOR_COUNT],
    ManipulatorPacketHeader* out_header) {
    if (!buf || !out_velocities) return false;
    ManipulatorPacketHeader hdr;
    if (!manipulator_unpack_packet_header(buf, buf_size, &hdr)) return false;

    if (hdr.command_id != set_joint_velocities) return false;
    if (hdr.content_size != sizeof(float) * MANIPULATOR_JOINT_MOTOR_COUNT) return false;

    if (!manipulator_unpack_joint_velocities_from_payload(
        buf + manipulator_packet_header_size(),
        hdr.content_size, out_velocities))
        return false;

    if (out_header) *out_header = hdr;
    return true;
}

const JointTrajectoryPoint* unpack_joint_trajectories_from_packet(
    const uint8_t* buf, size_t buf_size,
    uint32_t* out_point_count,
    ManipulatorPacketHeader* out_header) {
    if (!buf || !out_point_count) return NULL;
    ManipulatorPacketHeader hdr;
    if (!manipulator_unpack_packet_header(buf, buf_size, &hdr)) return NULL;
    if (hdr.command_id != set_joint_trajectories) return NULL;

    const uint8_t* payload = buf + manipulator_packet_header_size();
    uint32_t point_count = 0;
    const JointTrajectoryPoint* points =
        manipulator_unpack_joint_trajectories_from_payload(payload, hdr.content_size, &point_count);

    if (!points) return NULL;
    if (out_header) *out_header = hdr;
    *out_point_count = point_count;
    return points;
}

bool unpack_gripper_duties_from_packet(
    const uint8_t* buf, size_t buf_size,
    float out_duties[MANIPULATOR_GRIPPER_MOTOR_COUNT],
    ManipulatorPacketHeader* out_header) {
    if (!buf || !out_duties) return false;
    ManipulatorPacketHeader hdr;
    if (!manipulator_unpack_packet_header(buf, buf_size, &hdr)) return false;
    if (hdr.command_id != set_gripper_duties) return false;
    if (hdr.content_size != sizeof(float) * MANIPULATOR_GRIPPER_MOTOR_COUNT) return false;

    if (!manipulator_unpack_gripper_duties_from_payload(
        buf + manipulator_packet_header_size(),
        hdr.content_size, out_duties))
        return false;

    if (out_header) *out_header = hdr;
    return true;
}

bool unpack_manipulator_init_packet(
    const uint8_t* buf, size_t buf_size,
    ManipulatorInitPacket* out_init,
    ManipulatorPacketHeader* out_header) {
    if (!buf || !out_init) return false;
    ManipulatorPacketHeader hdr;
    if (!manipulator_unpack_packet_header(buf, buf_size, &hdr)) return false;

    if (hdr.command_id != init_device) return false;
    if (hdr.content_size != sizeof(ManipulatorInitPacket)) return false;

    if (!manipulator_unpack_init_packet_from_payload(
        buf + manipulator_packet_header_size(),
        hdr.content_size, out_init))
        return false;

    if (out_header) *out_header = hdr;
    return true;
}