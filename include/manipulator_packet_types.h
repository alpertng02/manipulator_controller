#ifndef __MANIPULATOR_PACKET_TYPES_H__
#define __MANIPULATOR_PACKET_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

    /* ==============================
     * Constants
     * ============================== */

#define MANIPULATOR_DEVICE_ID        (0x706F726E)
#define MANIPULATOR_JOINT_MOTOR_COUNT   (3u)
#define MANIPULATOR_GRIPPER_MOTOR_COUNT (3u)

     /* ==============================
      * Commands
      * ============================== */
    typedef enum {
        set_joint_velocities = 0,
        set_joint_trajectories = 1,
        set_gripper_duties = 2,
        set_joint_current_positions = 3,
        set_joint_pos_boundaries = 4,
        set_lowpass_fc = 5,
        set_pid_params = 6,
        set_pid_bounds = 7,
        set_ff_params = 8,
        set_gripper_max_duty = 9,
        set_feedback_hz = 11,
        set_control_hz = 12,

        running_mode_enable = 13,
        init_mode_enable = 14,

        request_device_state = 32,
        request_joint_feedback = 33,
        
        init_device = 42,
        
        joint_feedback = 64,
        device_state = 65
    } ManipulatorCommands;

    /* ==============================
     * Data Structures
     * ============================== */

#pragma pack(push, 1)
    typedef struct {
        float position;
        float velocity;
        float acceleration;
        float time_from_start_seconds;
    } JointTrajectoryPoint;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        uint32_t overwrite_pinout;

        uint8_t joint_dir_pins[3];
        uint8_t joint_pul_pins[3];

        uint8_t gripper_lpwm_pins[3];
        uint8_t gripper_rpwm_pins[3];

        uint8_t joint_swap_dirs[3];
        uint8_t gripper_swap_dirs[3];

        int32_t joint_initial_pos[3];

        int32_t max_joint_pos_boundaries[3];
        int32_t min_joint_pos_boundaries[3];

        float max_dutycycle;
        float lowpass_fc;
        float kp;
        float ki;
        float kd;
        float p_bound;
        float i_bound;
        float d_bound;

        float kv;
        float ka;
        float kj;

        float feedback_hz;
        float control_hz;
    } ManipulatorInitPacket;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        uint8_t device_is_init;
        uint8_t device_is_running;
    } ManipulatorDeviceState;
#pragma pack(pop)


#pragma pack(push, 1)
    typedef struct {
        float joint_positions[MANIPULATOR_JOINT_MOTOR_COUNT];
        float joint_velocities[MANIPULATOR_JOINT_MOTOR_COUNT];
        float gripper_pwm_duties[MANIPULATOR_GRIPPER_MOTOR_COUNT];
    } ManipulatorFeedback;
#pragma pack(pop)


#pragma pack(push, 1)
    typedef struct {
        uint32_t device_id;
        uint16_t command_id;
        uint16_t sequence;
        uint16_t content_size;
        uint64_t time_since_boot_us;
        uint32_t crc32;
    } ManipulatorPacketHeader;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        ManipulatorPacketHeader header;
        uint8_t payload[]; /* binary payload, little-endian encoded */
    } ManipulatorPacket;
#pragma pack(pop)


#ifdef __cplusplus
}
#endif

#endif // __MANIPULATOR_PACKET_TYPES_H__