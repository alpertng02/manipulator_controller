#ifndef __ICOMMUNICATION_PROTOCOL_H__
#define __ICOMMUNICATION_PROTOCOL_H__

#include <string>
#include <chrono>
#include <array>
#include <vector>

#include <manipulator_packets.h>

class ICommunicationProtocol {
public:

    virtual bool open(const std::string& dev) = 0;

    virtual bool is_open() = 0;

    virtual std::vector<std::string> list_all_devices() = 0;

    virtual int write_bytes(void* buffer, const unsigned int n_bytes) = 0;

    virtual int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) = 0;

    virtual bool send_command_packet(ManipulatorCommandPacket& packet);
    virtual bool send_init_packet(ManipulatorInitPacket& packet);
    
    virtual bool receive_motor_feedback(ManipulatorFeedbackPacket* packet, std::chrono::milliseconds timeout);
    virtual bool receive_state_feedback(ManipulatorStatePacket* packet, std::chrono::milliseconds timeout);
        
    virtual bool set_joint_velocities(const std::array<float, 3>& velocities);

    virtual bool set_joint_trajectories(
        const std::array<int32_t, 3>& target_positions,
        const std::array<float, 3>& max_velocities,
        const std::array<float, 3>& final_velocities,
        const std::array<float, 3>& accelerations,
        const std::array<float, 3>& deaccelerations,
        const std::array<float, 3>& acc_jerks,
        const std::array<float, 3>& dec_jerks);

    virtual bool set_gripper_dutycycles(const std::array<float, 3>& dutycycles);

    virtual bool set_joint_current_positions(const std::array<int32_t, 3>& current_positions);

    // Start / Stop controller
    virtual bool set_running_mode_enabled(bool enable);

    virtual bool set_init_mode_enabled(bool enable);

    virtual bool request_device_state();

    virtual void close() = 0;

    virtual ~ICommunicationProtocol() = 0;
};

#endif // __ICOMMUNICATION_PROTOCOL_H__