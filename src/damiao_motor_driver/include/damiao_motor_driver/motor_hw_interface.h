#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <damiao_motor_driver/motor_driver.h>

class MotorHWInterface : public hardware_interface::RobotHW
{
public:
    MotorHWInterface(ros::NodeHandle& nh);
    void read();   // 从电机读取状态
    void write();  // 发送控制命令

private:
    ros::NodeHandle nh_;
    MotorDriver driver_;

    static constexpr int N = 30;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;

    double cmd_pos_[N], cmd_vel_[N], cmd_eff_[N];
    double pos_[N], vel_[N], eff_[N];
};
