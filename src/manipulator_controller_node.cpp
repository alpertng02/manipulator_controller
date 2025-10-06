/**
 *@file mobility_control_node_usb.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-10-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "manipulator_packets.h"
#include "icommunication_protocol.hpp"
#include "usb_protocol.hpp"

class ManipulatorControlNode : public rclcpp::Node {

public:
    ManipulatorControlNode() : Node("manipulator_control_node") {

        init_parameters();

        joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(joint_trajectory_topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
            std::bind(&ManipulatorControlNode::joint_trajectory_callback, this, std::placeholders::_1));

        jointStatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default)));

        size_t jointCount = manipulator_joint_names_.size();
        manipulator_joint_states_.name.reserve(jointCount);
        manipulator_joint_states_.position.reserve(jointCount);
        manipulator_joint_states_.velocity.reserve(jointCount);
        manipulator_joint_states_.name.resize(jointCount);
        manipulator_joint_states_.position.resize(jointCount);
        manipulator_joint_states_.velocity.resize(jointCount);
        for (size_t i = 0; i < manipulator_joint_names_.size(); i++) {
            manipulator_joint_states_.name[i] = manipulator_joint_names_[i];
            manipulator_joint_states_.position[i] = 0.0;
            manipulator_joint_states_.velocity[i] = 0.0;
        }

        // Initialize timers but keep them disabled
        joint_state_timer_ = this->create_wall_timer(
            std::chrono::duration<double, std::milli>(1000.0 / feedback_rate_hz_),
            std::bind(&ManipulatorControlNode::publish_joint_states, this));
        joint_state_timer_->cancel();

        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<double, std::milli>(1000.0 / command_publish_rate_hz_),
            std::bind(&ManipulatorControlNode::command_controller, this));
        controller_timer_->cancel();

        if (protocol_type_ == "usb") {
            protocol_ = std::make_unique<UsbProtocol>();
        } else {
            RCLCPP_FATAL(this->get_logger(), "Unsupported protocol: %s", protocol_type_.c_str());
            throw std::runtime_error("Unsupported protocol");
        }
        communication_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(1000.0 / feedback_rate_hz_)),
            std::bind(&ManipulatorControlNode::communication_loop, this));

    }
private:


    std::unique_ptr<ICommunicationProtocol> protocol_;

    ManipulatorFeedbackPacket latest_feedback_ {};

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_subscriber_ {};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_ {};
    rclcpp::TimerBase::SharedPtr joint_state_timer_ {};
    rclcpp::TimerBase::SharedPtr controller_timer_ {};
    rclcpp::TimerBase::SharedPtr communication_timer_ {};

    sensor_msgs::msg::JointState manipulator_joint_states_ {};

    std::string protocol_type_ { "usb" };

    std::string joint_trajectory_topic_ { "/manipulator/joint_trajectory" };
    std::string joint_states_topic_ { "/manipulator/joint_states" };

    bool overwrite_pinout_ { true };
    std::vector<int64_t> joint_dir_pins_ { 3, 7, 9 };
    std::vector<int64_t> joint_pul_pins_ { 2, 6, 8 };
    std::vector<int64_t> gripper_lpwm_pins_ { 14, 10, 12 };
    std::vector<int64_t> gripper_rpwm_pins_ { 15, 11, 13 };

    std::vector<int64_t> joint_swap_dirs_ { 0, 0, 0 };
    std::vector<int64_t> gripper_swap_dirs_ { 0, 0, 0 };

    std::vector<double> joint_initial_positions_rads_ { 0, 0, 0 };

    std::vector<double> joint_min_pos_boundaries_rads_ { -M_PI, -M_PI, -M_PI };
    std::vector<double> joint_max_pos_boundaries_rads_ { M_PI, M_PI, M_PI };

    std::vector<double> joint_reductions_ { 50.0, 68.18181818, 2.0 };
    std::vector<double> joint_steps_per_revolutions_ { 51200.0, 51200.0, 51200.0 };

    bool use_joint_trajectories_ { false };

    double joint_deacceleration_ratio_ { 0.8f };

    std::vector<double> joint_max_acc_jerks_rads_ { 10.0f, 10.0f, 10.0f };
    std::vector<double> joint_max_dec_jerks_rads_ { 8.0f, 8.0f, 8.0f };

    std::vector<double> wheel_encoder_velocities_steps_sec {
        0.0, 0.0, 0.0, 0.0
    };

    std::vector<std::string> manipulator_joint_names_ {
        "joint_1",
        "joint_2",
        "joint_3",
        "gripper_1",
        "gripper_2",
        "gripper_3"
    };
    std::string base_frame_id_ { "base_link" };

    double max_pwm_dutycycle_ { 80.0 };
    double velocity_filter_cutoff_hz_ { 100.0 };

    double pid_kp_ { 0.0 };
    double pid_ki_ { 0.00 };
    double pid_kd_ { 0.0000 };

    double pid_p_bound_ { 10000.0f };
    double pid_i_bound_ { 5000.0f };
    double pid_d_bound_ { 5000.0f };

    double ff_kv_ { 1.0 };
    double ff_ka_ { 0.001 };
    double ff_kj_ { 0.000001 };

    double motor_control_rate_hz_ = 1000.0;
    double feedback_rate_hz_ = 200.0;
    double command_publish_rate_hz_ = 50.0;

    double joint_trajectory_timeout_sec_ = 0.5;

    int64_t device_id_ { MANIPULATOR_DEVICE_ID };
    std::string device_ { "/dev/ttyACM0" };

    rclcpp::Time prev_joint_trajectory_time_ { this->get_clock()->now() };
    rclcpp::Time prev_joint_states_time_ { this->get_clock()->now() };
    rclcpp::Time prev_log_time_ { this->get_clock()->now() };
    trajectory_msgs::msg::JointTrajectory joint_trajectory_;

    ManipulatorFeedbackPacket feedback_ {};

    bool connection_log_occurred_ = false;
    bool connect_to_device(uint32_t device_id) {
        if (!connection_log_occurred_) {
            RCLCPP_INFO(this->get_logger(), "Listing all available devices: ");
        }
        auto available_devices = protocol_->list_all_devices();

        if (available_devices.empty()) {
            if (!connection_log_occurred_) {
                RCLCPP_ERROR(this->get_logger(), "No devices available");
            }
        }
        for (const auto& device : available_devices) {
            RCLCPP_INFO(this->get_logger(), "\t%s", device.c_str());
        }

        ManipulatorStatePacket state_packet {};
        bool res = false;
        for (const auto& device : available_devices) {

            protocol_->close();
            res = protocol_->open(device);
            if (res) {
                RCLCPP_INFO(this->get_logger(), "Connected to %s", device.c_str());
                rclcpp::sleep_for(std::chrono::milliseconds(10));
                RCLCPP_INFO(this->get_logger(), "Requesting Device ID");
                res = protocol_->request_device_state();
                if (res) {
                    RCLCPP_INFO(this->get_logger(), "Device ID Requested");
                    res = protocol_->receive_state_feedback(&state_packet, std::chrono::milliseconds(10));
                    if (!res) {
                        RCLCPP_ERROR(this->get_logger(), "Device did not send ID.");
                        res = protocol_->receive_motor_feedback(&feedback_, std::chrono::milliseconds(10));
                        if (res) {
                            RCLCPP_INFO(this->get_logger(), "Device is already running");
                            res = protocol_->set_init_mode_enabled(true);
                            if (res) {
                                RCLCPP_INFO(this->get_logger(), "Init mode enabled");
                                res = init_arm_controller();
                                connection_log_occurred_ = false;
                            } else {
                                RCLCPP_ERROR(this->get_logger(), "Could not enable init mode");
                            }
                            break;
                        }
                    } else if (state_packet.device_id == device_id) {
                        RCLCPP_INFO(this->get_logger(), "Device ID matched: %u", state_packet.device_id);
                        RCLCPP_INFO(this->get_logger(), "Enabling init mode");
                        device_ = device;
                        res = protocol_->set_init_mode_enabled(true);
                        if (res) {
                            RCLCPP_INFO(this->get_logger(), "Init mode enabled");
                            res = init_arm_controller();
                            connection_log_occurred_ = false;

                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Could not enable init mode");
                        }
                        break;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Device ID did not match: %u", state_packet.device_id);
                    }

                } else {
                    RCLCPP_ERROR(this->get_logger(), "Device state request could not be sent.");
                }
            }
        }
        if (!res) {
            if (!connection_log_occurred_) {
                RCLCPP_ERROR(this->get_logger(), "Device did not found");
            }
            connection_log_occurred_ = true;
            protocol_->close();
            return false;
        }
        return res;
    }

    void enable_timers(bool enable) {
        if (enable) {
            if (joint_state_timer_->is_canceled()) {
                RCLCPP_INFO(this->get_logger(), "Enabling joint state and control timers.");
                joint_state_timer_->reset();
                controller_timer_->reset();
            }
        } else {
            if (!joint_state_timer_->is_canceled()) {
                RCLCPP_WARN(this->get_logger(), "Disabling joint state and control timers.");
                joint_state_timer_->cancel();
                controller_timer_->cancel();
            }
        }
    }

    bool communication_established_ = false;

    void communication_loop() {
        if (protocol_->is_open()) {
            ManipulatorFeedbackPacket fb;
            if (protocol_->receive_motor_feedback(&fb, std::chrono::milliseconds(1))) {
                latest_feedback_ = fb;
            }
        } else {

            if (communication_established_) {
                RCLCPP_ERROR(this->get_logger(), "Device connection lost. Trying to reconnect to %s", device_.c_str());
                communication_established_ = false;
            }

            enable_timers(false);

            bool res = connect_to_device(device_id_);
            if (res) {
                communication_established_ = true;
                enable_timers(true);
            }
        }
    }

    void joint_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        if (msg != nullptr) {
            prev_joint_trajectory_time_ = this->get_clock()->now();
            joint_trajectory_ = *msg;
        }
    }

    std::vector<double> joint_steps_to_rads(const std::vector<double>& steps) {
        std::vector<double> joint_rads {};
        for (size_t i = 0; i < joint_steps_per_revolutions_.size(); i++) {
            double rads = (steps[i] * 2.0 * M_PI) / (joint_steps_per_revolutions_[i] * joint_reductions_[i]);
            joint_rads.push_back(rads);
        }
        return joint_rads;
    }

    std::vector<double> joint_rads_to_steps(const std::vector<double>& rads) {
        std::vector<double> joint_steps {};
        for (size_t i = 0; i < joint_steps_per_revolutions_.size(); i++) {
            double steps = (rads[i] * joint_steps_per_revolutions_[i] * joint_reductions_[i]) / (2.0 * M_PI);
            joint_steps.push_back(steps);
        }
        return joint_steps;
    }

    void publish_joint_states() {
        rclcpp::Time current_time = this->get_clock()->now();
        prev_joint_states_time_ = current_time;

        feedback_ = latest_feedback_;
        std::vector<double> joint_positions_steps { feedback_.joint_positions, feedback_.joint_positions + 3 };
        std::vector<double> joint_velocities_steps { feedback_.joint_velocities, feedback_.joint_velocities + 3 };

        std::vector<double> joint_positions_rads { joint_steps_to_rads(joint_positions_steps) };
        std::vector<double> joint_velocities_rads { joint_steps_to_rads(joint_velocities_steps) };

        manipulator_joint_states_.header.frame_id = base_frame_id_;
        manipulator_joint_states_.header.stamp = current_time;
        for (size_t i = 0; i < 3; i++) {
            manipulator_joint_states_.velocity[i] = joint_velocities_rads[i];
            manipulator_joint_states_.position[i] = joint_positions_rads[i];
            manipulator_joint_states_.velocity[i + 3] = feedback_.gripper_pwm_duties[i];
        }
        jointStatePublisher_->publish(manipulator_joint_states_);

    }

    std::vector<float> prev_velocities { 0, 0, 0 };
    void command_controller() {
        auto current_time = this->get_clock()->now();

        if (joint_trajectory_.points.empty()) return;

        const auto& point = joint_trajectory_.points[0];
        if ((current_time - prev_joint_trajectory_time_).seconds() < joint_trajectory_timeout_sec_) {
            if (point.velocities.size() < manipulator_joint_states_.name.size()) {
                return;
            }

            std::vector<double> joint_velocities_rads { point.velocities.begin(), point.velocities.begin() + 3 };
            std::vector<double> joint_velocities_steps { joint_rads_to_steps(joint_velocities_rads) };

            std::vector<double> gripper_duties { point.velocities.begin() + 3, point.velocities.begin() + 6 };
            // Only send command if we have 3 joints
            bool res = true;
            if (use_joint_trajectories_) {

                std::vector<double> joint_positions_rads { point.positions };
                std::vector<double> joint_accelerations_rads { point.accelerations };

                std::vector<double> joint_positions_steps { joint_rads_to_steps(joint_positions_rads) };
                std::vector<double> joint_accelerations_steps { joint_rads_to_steps(joint_accelerations_rads) };
                std::vector<double> joint_acc_jerks_steps { joint_rads_to_steps(joint_max_acc_jerks_rads_) };
                std::vector<double> joint_dec_jerks_steps { joint_rads_to_steps(joint_max_dec_jerks_rads_) };


                std::array<int32_t, 3> positions { static_cast<int32_t>(lround(joint_positions_steps[0])),
                    static_cast<int32_t>(lround(joint_positions_steps[1])),
                    static_cast<int32_t>(lround(joint_positions_steps[2])) };

                std::array<float, 3> velocities { static_cast<float>(joint_velocities_steps[0]),
                    static_cast<float>(joint_velocities_steps[1]),
                    static_cast<float>(joint_velocities_steps[2]) };

                std::array<float, 3> final_velocities {};
                for (size_t i = 0; i < velocities.size(); i++) {
                    if (velocities[i] == 0.0f) {
                        final_velocities[i] = 0.0f;
                        velocities[i] = prev_velocities[i];
                    } else {
                        final_velocities[i] = velocities[i];
                    }
                    prev_velocities[i] = velocities[i];
                }

                std::array<float, 3> accelerations { static_cast<float>(joint_accelerations_steps[0]),
                    static_cast<float>(joint_accelerations_steps[1]),
                    static_cast<float>(joint_accelerations_steps[2]) };

                std::array<float, 3> deaccelerations { static_cast<float>(joint_deacceleration_ratio_) * accelerations[0],
                    static_cast<float>(joint_deacceleration_ratio_) * accelerations[1],
                    static_cast<float>(joint_deacceleration_ratio_) * accelerations[2] };

                std::array<float, 3> acc_jerks { static_cast<float>(joint_acc_jerks_steps[0]),
                    static_cast<float>(joint_acc_jerks_steps[1]),
                    static_cast<float>(joint_acc_jerks_steps[2]) };

                std::array<float, 3> dec_jerks { static_cast<float>(joint_dec_jerks_steps[0]),
                    static_cast<float>(joint_dec_jerks_steps[1]),
                    static_cast<float>(joint_dec_jerks_steps[2]) };


                res &= protocol_->set_joint_trajectories(
                    positions,
                    velocities,
                    final_velocities,  // final velocities, could be separate if needed
                    accelerations,
                    deaccelerations, // deaccelerations, could be separate
                    acc_jerks,
                    dec_jerks
                );
            } else {

                std::array<float, 3> velocities { static_cast<float>(joint_velocities_steps[0]),
                    static_cast<float>(joint_velocities_steps[1]),
                    static_cast<float>(joint_velocities_steps[2]) };

                res &= protocol_->set_joint_velocities(velocities);
            }
            std::array<float, 3> duties { static_cast<float>(gripper_duties[0]),
                static_cast<float>(gripper_duties[1]),
                static_cast<float>(gripper_duties[2]) };

            res &= protocol_->set_gripper_dutycycles(duties);
            if (!res) {
                enable_timers(false);
            }
        }
    }


    bool init_arm_controller() {
        try {

            ManipulatorInitPacket pkt {};

            pkt.overwrite_pinout = overwrite_pinout_ ? 1 : 0;
            auto joint_initial_pos_steps = joint_rads_to_steps(joint_initial_positions_rads_);
            auto joint_max_pos_boundaries_steps = joint_rads_to_steps(joint_max_pos_boundaries_rads_);
            auto joint_min_pos_boundaries_steps = joint_rads_to_steps(joint_min_pos_boundaries_rads_);

            for (int i = 0; i < 3; i++) {
                pkt.joint_dir_pins[i] = joint_dir_pins_[i];
                pkt.joint_pul_pins[i] = joint_pul_pins_[i];
                pkt.gripper_lpwm_pins[i] = gripper_lpwm_pins_[i];
                pkt.gripper_rpwm_pins[i] = gripper_rpwm_pins_[i];

                pkt.joint_swap_dirs[i] = joint_swap_dirs_[i];
                pkt.gripper_swap_dirs[i] = gripper_swap_dirs_[i];

                pkt.joint_initial_pos[i] = static_cast<int32_t>(lround(joint_initial_pos_steps[i]));

                pkt.max_joint_pos_boundaries[i] = static_cast<int32_t>(lround(joint_max_pos_boundaries_steps[i]));
                pkt.min_joint_pos_boundaries[i] = static_cast<int32_t>(lround(joint_min_pos_boundaries_steps[i]));
            }

            pkt.max_dutycycle = max_pwm_dutycycle_;
            pkt.lowpass_fc = velocity_filter_cutoff_hz_;

            pkt.kp = pid_kp_;
            pkt.ki = pid_ki_;
            pkt.kd = pid_kd_;

            pkt.p_bound = pid_p_bound_;
            pkt.i_bound = pid_i_bound_;
            pkt.d_bound = pid_d_bound_;

            pkt.kv = ff_kv_;
            pkt.ka = ff_ka_;
            pkt.kj = ff_kj_;

            pkt.feedback_hz = feedback_rate_hz_;
            pkt.control_hz = motor_control_rate_hz_;

            if (!protocol_->send_init_packet(pkt)) {
                return false;
            }

            if (pkt.overwrite_pinout) {
                RCLCPP_INFO(this->get_logger(), "Overwritten Pinout:");

                RCLCPP_INFO(this->get_logger(), "Joint Dir Pins: %d, %d, %d",
                    (int) pkt.joint_dir_pins[0], (int) pkt.joint_dir_pins[1], (int) pkt.joint_dir_pins[2]);
                RCLCPP_INFO(this->get_logger(), "Joint Pul Pins: %d, %d, %d",
                    (int) pkt.joint_pul_pins[0], (int) pkt.joint_pul_pins[1], (int) pkt.joint_pul_pins[2]);

                RCLCPP_INFO(this->get_logger(), "Gripper LPWM Pins: %d, %d, %d",
                    (int) pkt.gripper_lpwm_pins[0], (int) pkt.gripper_lpwm_pins[1], (int) pkt.gripper_lpwm_pins[2]);
                RCLCPP_INFO(this->get_logger(), "Gripper RPWM Pins: %d, %d, %d",
                    (int) pkt.gripper_rpwm_pins[0], (int) pkt.gripper_rpwm_pins[1], (int) pkt.gripper_rpwm_pins[2]);
            }

            RCLCPP_INFO(this->get_logger(), "Joint Directions: %s,%s,%s",
                pkt.joint_swap_dirs[0] ? "FL: Swapped " : "Not Swapped ",
                pkt.joint_swap_dirs[1] ? "FR: Swapped " : "Not Swapped ",
                pkt.joint_swap_dirs[2] ? "BR: Swapped " : "Not Swapped ");

            RCLCPP_INFO(this->get_logger(), "Gripper Directions: %s,%s,%s",
                pkt.gripper_swap_dirs[0] ? "FL: Swapped " : "Not Swapped ",
                pkt.gripper_swap_dirs[1] ? "FR: Swapped " : "Not Swapped ",
                pkt.gripper_swap_dirs[2] ? "BR: Swapped " : "Not Swapped ");

            RCLCPP_INFO(this->get_logger(), "Joint Initial Positions Steps: %d, %d, %d",
                (int) pkt.joint_initial_pos[0], (int) pkt.joint_initial_pos[1], (int) pkt.joint_initial_pos[2]);

            RCLCPP_INFO(this->get_logger(), "Joint Positions Boundaries : [%d, %d], [%d, %d], [%d, %d]",
                (int) pkt.min_joint_pos_boundaries[0], (int) pkt.max_joint_pos_boundaries[0],
                (int) pkt.min_joint_pos_boundaries[1], (int) pkt.max_joint_pos_boundaries[1],
                (int) pkt.min_joint_pos_boundaries[2], (int) pkt.max_joint_pos_boundaries[2]
            );

            RCLCPP_INFO(this->get_logger(), "Setted maximum Gripper PWM dutycycle: %f", pkt.max_dutycycle);
            RCLCPP_INFO(this->get_logger(), "Setted new velocity filter cutoff: %f Hz", pkt.lowpass_fc);

            RCLCPP_INFO(this->get_logger(), "Setted new Feed Forward Parameters: kv:%f, ka:%f, kj:%f", pkt.kv, pkt.ka, pkt.kj);
            RCLCPP_INFO(this->get_logger(), "Setted new PID parameters: kp:%f, ki:%f, kd:%f", pkt.kp, pkt.ki, pkt.kd);
            RCLCPP_INFO(this->get_logger(), "Setted new PID boundaries: p:%f, i:%f, d:%f", pkt.p_bound, pkt.i_bound, pkt.d_bound);

            RCLCPP_INFO(this->get_logger(), "Setted new motor control rate: %f Hz", pkt.control_hz);
            RCLCPP_INFO(this->get_logger(), "Setted new feedback rate: %f Hz", pkt.feedback_hz);

            return true;
        } catch (std::runtime_error& err) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize device");
            return false;
        }
    }

    void init_parameters() {

        protocol_type_ = this->declare_parameter("protocol", protocol_type_);
        device_id_ = this->declare_parameter("device_id", device_id_);

        command_publish_rate_hz_ = this->declare_parameter("command_publish_rate", command_publish_rate_hz_);
        feedback_rate_hz_ = this->declare_parameter("feedback_rate", feedback_rate_hz_);
        motor_control_rate_hz_ = this->declare_parameter("motor_control_rate", motor_control_rate_hz_);

        joint_trajectory_topic_ = this->declare_parameter("joint_trajectory_topic", joint_trajectory_topic_);
        joint_states_topic_ = this->declare_parameter("joint_states_topic", joint_states_topic_);

        manipulator_joint_names_ = this->declare_parameter("manipulator_joint_names", manipulator_joint_names_);

        max_pwm_dutycycle_ = this->declare_parameter("max_pwm_dutycycle", max_pwm_dutycycle_);
        velocity_filter_cutoff_hz_ = this->declare_parameter("velocity_filter_cutoff", velocity_filter_cutoff_hz_);

        pid_kp_ = this->declare_parameter("pid_kp", pid_kp_);
        pid_ki_ = this->declare_parameter("pid_ki", pid_ki_);
        pid_kd_ = this->declare_parameter("pid_kd", pid_kd_);

        pid_p_bound_ = this->declare_parameter("pid_p_bound", pid_p_bound_);
        pid_i_bound_ = this->declare_parameter("pid_i_bound", pid_i_bound_);
        pid_d_bound_ = this->declare_parameter("pid_d_bound", pid_d_bound_);

        ff_kv_ = this->declare_parameter("ff_kv", ff_kv_);
        ff_ka_ = this->declare_parameter("ff_ka", ff_ka_);
        ff_kj_ = this->declare_parameter("ff_kj", ff_kj_);

        overwrite_pinout_ = this->declare_parameter("overwrite_pinout", overwrite_pinout_);
        joint_dir_pins_ = this->declare_parameter("joint_dir_pins", joint_dir_pins_);
        joint_pul_pins_ = this->declare_parameter("joint_pul_pins", joint_pul_pins_);
        gripper_lpwm_pins_ = this->declare_parameter("gripper_lpwm_pins", gripper_lpwm_pins_);
        gripper_rpwm_pins_ = this->declare_parameter("gripper_rpwm_pins", gripper_rpwm_pins_);

        joint_swap_dirs_ = this->declare_parameter("joint_swap_dirs", joint_swap_dirs_);
        gripper_swap_dirs_ = this->declare_parameter("gripper_swap_dirs", gripper_swap_dirs_);

        joint_initial_positions_rads_ = this->declare_parameter("joint_initial_positions", joint_initial_positions_rads_);
        joint_min_pos_boundaries_rads_ = this->declare_parameter("joint_min_pos_boundaries", joint_min_pos_boundaries_rads_);
        joint_max_pos_boundaries_rads_ = this->declare_parameter("joint_max_pos_boundaries", joint_max_pos_boundaries_rads_);

        joint_deacceleration_ratio_ = this->declare_parameter("joint_deacceleration_ratio", joint_deacceleration_ratio_);

        joint_max_acc_jerks_rads_ = this->declare_parameter("joint_max_acc_jerks", joint_max_acc_jerks_rads_);
        joint_max_dec_jerks_rads_ = this->declare_parameter("joint_max_dec_jerks", joint_max_dec_jerks_rads_);

        base_frame_id_ = this->declare_parameter("base_frame_id", base_frame_id_);

        joint_trajectory_timeout_sec_ = this->declare_parameter("joint_trajectory_timeout", joint_trajectory_timeout_sec_);
    }

};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ManipulatorControlNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}