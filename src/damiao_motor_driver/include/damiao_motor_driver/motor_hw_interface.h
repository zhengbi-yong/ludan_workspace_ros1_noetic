#pragma once

#include <dynamic_reconfigure/server.h>
#include <controller_manager/controller_manager.h>
#include <damiao_motor_driver/DriverLimitsConfig.h>
#include <damiao_motor_core/motor_driver_core.h>
#include <damiao_motor_core/serial_transport.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <map>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_srvs/Trigger.h>

namespace damiao_motor_driver {
class MotorHWInterface : public hardware_interface::RobotHW
{
public:
    MotorHWInterface();
    MotorHWInterface(const ros::NodeHandle& nh, std::shared_ptr<damiao_motor_core::Transport> transport = nullptr);
    void read();   // 从电机读取状态
    void write();  // 发送控制命令
    ros::Time getLastFeedbackTime() const;
    ros::Time getLastCommandUpdateTime() const;
    void sendSafeMode();
    void stopDriver();
    bool goZero(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);

private:
    ros::ServiceServer zero_srv_;   

    struct JointConfig
    {
        std::string name;
        int id;
        double kp;
        double kd;
    };

    void load_joint_config();
    void refresh_gain_map(const std::string& param_name, std::map<std::string, double>& output_map) const;

    ros::NodeHandle nh_;
    std::shared_ptr<damiao_motor_core::MotorDriverCore> driver_core_;
    damiao_motor_core::DriverConfig config_;
    std::string joint_prefix_;
    std::mutex limits_mutex_;
    dynamic_reconfigure::Server<damiao_motor_driver::DriverLimitsConfig> dr_srv_;

    struct CommandLimits
    {
        double kp;
        double kd;
        double torque;
        double velocity;
        double position;
    } limits_;

    double default_kp_;
    double default_kd_;

    std::vector<JointConfig> joints_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    hardware_interface::EffortJointInterface jnt_eff_interface_;

    std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_;
    std::vector<double> pos_, vel_, eff_;

    mutable std::mutex cmd_mutex_;
    ros::Time last_command_update_;
};
}