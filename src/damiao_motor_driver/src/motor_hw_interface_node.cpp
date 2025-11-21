#include <boost/make_shared.hpp>
#include <controller_manager/controller_manager.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <hardware_interface/robot_hw.h>
#include <limits>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "damiao_motor_driver/motor_hw_interface.h"

using damiao_motor_driver::MotorHWInterface;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hardware");
    
    ros::NodeHandle private_nh("~");

    // ⭐ 最重要一步：把 private_nh 传进来（否则 joint 列表加载不到）
    auto motor_hw = boost::make_shared<MotorHWInterface>(private_nh);

    // ⭐ controller_manager 必须用 private_nh
    controller_manager::ControllerManager cm(motor_hw.get(), private_nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    double loop_hz;
    private_nh.param("loop_hz", loop_hz, 500.0);

    ros::Rate rate(loop_hz);
    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - last_time;
        last_time = now;

        motor_hw->read();
        cm.update(now, period);
        motor_hw->write();

        rate.sleep();
    }

    return 0;
}
