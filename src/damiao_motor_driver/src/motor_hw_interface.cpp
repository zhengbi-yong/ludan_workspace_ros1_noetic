#include "damiao_motor_driver/motor_hw_interface.h"

#include <algorithm>

MotorHWInterface::MotorHWInterface(ros::NodeHandle& nh)
    : nh_(nh), driver_(nh)
{
    limits_.kp = MITProtocol::KP_MAX;
    limits_.kd = MITProtocol::KD_MAX;
    limits_.torque = MITProtocol::T_MAX;
    limits_.velocity = MITProtocol::V_MAX;
    limits_.position = MITProtocol::P_MAX;

    nh_.param("kp_limit", limits_.kp, limits_.kp);
    nh_.param("kd_limit", limits_.kd, limits_.kd);
    nh_.param("torque_limit", limits_.torque, limits_.torque);
    nh_.param("velocity_limit", limits_.velocity, limits_.velocity);
    nh_.param("position_limit", limits_.position, limits_.position);

    driver_.start();

    for (int i = 0; i < N; i++)
    {
        hardware_interface::JointStateHandle state_handle(
            "joint_" + std::to_string(i),
            &pos_[i], &vel_[i], &eff_[i]
        );
        jnt_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(state_handle, &cmd_pos_[i]);
        jnt_pos_interface_.registerHandle(pos_handle);

        hardware_interface::JointHandle vel_handle(state_handle, &cmd_vel_[i]);
        jnt_vel_interface_.registerHandle(vel_handle);

        hardware_interface::JointHandle eff_handle(state_handle, &cmd_eff_[i]);
        jnt_eff_interface_.registerHandle(eff_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_eff_interface_);
}

void MotorHWInterface::read()
{
    auto states = driver_.get_states();
    const size_t available = states.size();
    for (int i = 0; i < N; i++) {
        if (static_cast<size_t>(i) < available) {
            pos_[i] = states[i].pos;
            vel_[i] = states[i].vel;
            eff_[i] = states[i].tor;
        }
    }
}

void MotorHWInterface::write()
{
    auto clamp_value = [](double value, double min_val, double max_val, bool& limited) {
        double clamped = std::min(std::max(value, min_val), max_val);
        if (clamped != value) {
            limited = true;
        }
        return clamped;
    };

    for (int i = 0; i < N; i++) {
        bool limited = false;
        const double pos_min = std::max(-limits_.position, static_cast<double>(MITProtocol::P_MIN));
        const double pos_max = std::min(limits_.position, static_cast<double>(MITProtocol::P_MAX));
        const double vel_min = std::max(-limits_.velocity, static_cast<double>(MITProtocol::V_MIN));
        const double vel_max = std::min(limits_.velocity, static_cast<double>(MITProtocol::V_MAX));
        const double tor_min = std::max(-limits_.torque, static_cast<double>(MITProtocol::T_MIN));
        const double tor_max = std::min(limits_.torque, static_cast<double>(MITProtocol::T_MAX));
        const double kp_min = static_cast<double>(MITProtocol::KP_MIN);
        const double kp_max = std::min(limits_.kp, static_cast<double>(MITProtocol::KP_MAX));
        const double kd_min = static_cast<double>(MITProtocol::KD_MIN);
        const double kd_max = std::min(limits_.kd, static_cast<double>(MITProtocol::KD_MAX));

        double pos_cmd = clamp_value(cmd_pos_[i], pos_min, pos_max, limited);
        double vel_cmd = clamp_value(cmd_vel_[i], vel_min, vel_max, limited);
        double eff_cmd = clamp_value(cmd_eff_[i], tor_min, tor_max, limited);
        double kp_cmd = clamp_value(20.0, kp_min, kp_max, limited);
        double kd_cmd = clamp_value(1.0, kd_min, kd_max, limited);

        MotorDriver::CommandStatus status = driver_.send_cmd(i, pos_cmd, vel_cmd, kp_cmd, kd_cmd, eff_cmd);
        if (limited || status != MotorDriver::CommandStatus::Ok) {
            ROS_ERROR_THROTTLE(1.0, "Command to motor %d limited or failed, sending zero torque", i);
            driver_.send_cmd(i, pos_cmd, 0.0, 0.0, 0.0, 0.0);
        }
    }
}
