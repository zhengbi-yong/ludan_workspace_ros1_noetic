#include "damiao_motor_driver/motor_hw_interface.h"

MotorHWInterface::MotorHWInterface(ros::NodeHandle& nh)
    : nh_(nh), driver_(nh)
{
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
    for (int i = 0; i < N; i++) {
        auto &m = driver_.get_states();
        pos_[i] = m[i].pos;
        vel_[i] = m[i].vel;
        eff_[i] = m[i].tor;
    }
}

void MotorHWInterface::write()
{
    for (int i = 0; i < N; i++) {
        driver_.send_cmd(i, cmd_pos_[i], cmd_vel_[i], 20, 1, cmd_eff_[i]);
    }
}
