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

#include "manipulator_packets.hpp"
#include "usb_device.hpp"

class ManipulatorControlNode : public rclcpp::Node {

public:
    ManipulatorControlNode() : Node("manipulator_control_node") {

        init_parameters();

        joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(joint_trajectory_topic_,
            rclcpp::SensorDataQoS(), std::bind(&ManipulatorControlNode::joint_trajectory_callback, this, std::placeholders::_1));

        jointStatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_,
            rclcpp::SystemDefaultsQoS());

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
            device_ = std::make_unique<UsbManipulatorDevice>();
        } else {
            RCLCPP_FATAL(this->get_logger(), "Unsupported protocol: %s", protocol_type_.c_str());
            throw std::runtime_error("Unsupported protocol");
        }

        device_connection_timer_ = this->create_wall_timer(
            std::chrono::duration<double, std::milli>(1000.0 * reconnection_retry_period_sec_),
            std::bind(&ManipulatorControlNode::device_connection_callback, this));

    }
private:

    std::unique_ptr<IManipulatorDevice> device_;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_subscriber_ {};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_ {};
    rclcpp::TimerBase::SharedPtr joint_state_timer_ {};
    rclcpp::TimerBase::SharedPtr controller_timer_ {};
    rclcpp::TimerBase::SharedPtr device_connection_timer_ {};

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
    std::vector<double> joint_steps_per_revolutions_ { 20000.0, 12800.0, 51200.0 };
    static constexpr double stepper_max_velocity_steps_ { 200000.0 };

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
    double feedback_rate_hz_ = 50.0;
    double command_publish_rate_hz_ = 200.0;
    double reconnection_retry_period_sec_ = 1.0;

    double joint_trajectory_timeout_sec_ = 0.5;

    int64_t device_id_ { MANIPULATOR_DEVICE_ID };

    rclcpp::Time prev_joint_trajectory_time_ { this->get_clock()->now() };
    rclcpp::Time prev_joint_states_time_ { this->get_clock()->now() };
    trajectory_msgs::msg::JointTrajectory joint_trajectory_;
    trajectory_msgs::msg::JointTrajectory prev_joint_trajectory_;

    manipulator::packets::Feedback feedback_ {};


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

    bool try_connect_device(std::chrono::milliseconds timeout) {

        auto available_ports = device_->list_all_ports();

        if (available_ports.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000, "No ports available");
            return false;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Available ports:");
        for (const auto& port : available_ports) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "\t%s\n", port.c_str());
        }

        for (const auto& port : available_ports) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Trying to connect to port: %s", port.c_str());
            if (!device_->open(port)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Port %s cannot open!", port.c_str());
                continue;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Connected to port: %s", port.c_str());
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Trying to initialize device...");
            if (device_->init_device(this->get_init_packet(), timeout)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Device is initialized!");
                return true;
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 5000, "Could not initialize device...");
                device_->close();
            }
        }
        return false;
    }

    void device_connection_callback() {
        if (!device_->is_open()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Device Disconnected");
            enable_timers(false);
            if (try_connect_device(std::chrono::milliseconds(200))) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Device Connected");
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

        try {
            feedback_ = device_->receive_joint_feedback(std::chrono::milliseconds(5)).feedback;

            std::vector<double> joint_positions_steps { feedback_.joint_positions, feedback_.joint_positions + 3 };
            std::vector<double> joint_velocities_steps { feedback_.joint_velocities, feedback_.joint_velocities + 3 };

            std::vector<double> joint_positions_rads { joint_steps_to_rads(joint_positions_steps) };
            std::vector<double> joint_velocities_rads { joint_steps_to_rads(joint_velocities_steps) };

            manipulator_joint_states_.header.frame_id = base_frame_id_;
            manipulator_joint_states_.header.stamp = current_time;
            for (size_t i = 0; i < 3; i++) {
                manipulator_joint_states_.velocity[i] = joint_velocities_rads[i];
                manipulator_joint_states_.position[i] = joint_positions_rads[i];
                joint_initial_positions_rads_[i] = joint_positions_rads[i];
                manipulator_joint_states_.velocity[i + 3] = feedback_.gripper_pwm_duties[i];
            }
            jointStatePublisher_->publish(manipulator_joint_states_);
        } catch (std::runtime_error& err) {
            RCLCPP_WARN(this->get_logger(), "Could not receive joint feedbacks");
        }

    }

    std::vector<float> prev_velocities { 0, 0, 0 };
    void command_controller() {
        auto current_time = this->get_clock()->now();

        if (joint_trajectory_.points.empty()) return;

        const auto& point = joint_trajectory_.points[0];
        if ((current_time - prev_joint_trajectory_time_).seconds() < joint_trajectory_timeout_sec_) {
            if (point.velocities.size() < manipulator_joint_states_.name.size()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Trajectory has insufficient velocity components");
                return;
            }

            std::vector<double> joint_velocities_rads { point.velocities.begin(), point.velocities.begin() + 3 };
            std::vector<double> joint_velocities_steps { joint_rads_to_steps(joint_velocities_rads) };

            std::vector<double> gripper_duties { point.velocities.begin() + 3, point.velocities.begin() + 6 };
            // Only send command if we have 3 joints
            bool res = true;

            std::array<float, 3> velocities { static_cast<float>(joint_velocities_steps[0]),
                static_cast<float>(joint_velocities_steps[1]),
                static_cast<float>(joint_velocities_steps[2]) };

            if (point.accelerations.size() >= MANIPULATOR_JOINT_MOTOR_COUNT) {

                std::vector<double> joint_accelerations_rads { point.accelerations.begin(), point.accelerations.begin() + 3 };
                std::vector<double> joint_accelerations_steps { joint_rads_to_steps(joint_accelerations_rads) };
                for (size_t i = 0; i < joint_accelerations_steps.size(); i++) {
                    double max_velocity_change = joint_accelerations_steps[i] / command_publish_rate_hz_;
                    if (velocities[i] > prev_velocities[i] + max_velocity_change) {
                        velocities[i] = prev_velocities[i] + max_velocity_change;
                    } else if (velocities[i] < prev_velocities[i] - max_velocity_change) {
                        velocities[i] = prev_velocities[i] - max_velocity_change;
                    }
                }
            }

            for (size_t i = 0; i < velocities.size(); i++) {
                velocities[i] = std::clamp(velocities[i],
                    -static_cast<float>(stepper_max_velocity_steps_),
                    static_cast<float>(stepper_max_velocity_steps_));
            }

            for (size_t i = 0; i < prev_velocities.size(); i++) {
                prev_velocities[i] = velocities[i];
            }

            res &= device_->send_joint_velocities(velocities);

            prev_joint_trajectory_ = joint_trajectory_;

            std::array<float, 3> duties { static_cast<float>(gripper_duties[0]),
                static_cast<float>(gripper_duties[1]),
                static_cast<float>(gripper_duties[2]) };

            res &= device_->send_gripper_dutycycles(duties);
            if (!res) {
                enable_timers(false);
            }
        }
    }

    manipulator::packets::InitPacket get_init_packet() {
        manipulator::packets::InitPacket pkt {};

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

        return pkt;
    }

    void init_parameters() {

        protocol_type_ = this->declare_parameter("protocol", protocol_type_);
        device_id_ = this->declare_parameter("device_id", device_id_);

        command_publish_rate_hz_ = this->declare_parameter("command_publish_rate", command_publish_rate_hz_);
        feedback_rate_hz_ = this->declare_parameter("feedback_rate", feedback_rate_hz_);
        motor_control_rate_hz_ = this->declare_parameter("motor_control_rate", motor_control_rate_hz_);

        reconnection_retry_period_sec_ = this->declare_parameter("reconnection_retry_period", reconnection_retry_period_sec_);

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

        joint_reductions_ = this->declare_parameter("joint_reductions", joint_reductions_);
        joint_steps_per_revolutions_ = this->declare_parameter("joint_steps_per_revolutions", joint_steps_per_revolutions_);

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