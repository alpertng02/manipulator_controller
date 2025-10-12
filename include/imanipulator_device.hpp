#ifndef __IMANIPULATOR_DEVICE_HPP__
#define __IMANIPULATOR_DEVICE_HPP__

#include <string>
#include <chrono>
#include <array>
#include <vector>

#include <manipulator_packets.hpp>

class IManipulatorDevice {
public:

    virtual bool open(const std::string& dev) = 0;

    virtual bool is_open() = 0;

    virtual std::vector<std::string> list_all_ports() = 0;

    virtual void close() = 0;

    virtual bool init_device(const manipulator::packets::InitPacket& init_packet, std::chrono::milliseconds timeout);

    virtual bool send_joint_velocities(const std::array<float, manipulator::packets::k_joint_motor_count>& velocities);

    virtual bool send_gripper_dutycycles(const std::array<float, manipulator::packets::k_gripper_motor_count>& dutycycles);

    virtual bool send_init_packet(const manipulator::packets::InitPacket& init_packet);

    virtual bool send_init_mode_enable(const bool enable);
    
    bool send_running_mode_enable(const bool enable);
    
    virtual std::vector<uint8_t> receive_packet(std::chrono::milliseconds timeout);

    virtual manipulator::packets::FeedbackPacket receive_joint_feedback(std::chrono::milliseconds timeout);

    virtual manipulator::packets::DeviceStatePacket receive_device_state(std::chrono::milliseconds timeout);

    virtual ~IManipulatorDevice() = 0;

private:
    virtual int write_bytes(void* buffer, const unsigned int n_bytes) = 0;

    virtual int read_bytes(void* buffer, int max_length, std::chrono::milliseconds timeout) = 0;
};

#endif // __IMANIPULATOR_DEVICE_HPP__