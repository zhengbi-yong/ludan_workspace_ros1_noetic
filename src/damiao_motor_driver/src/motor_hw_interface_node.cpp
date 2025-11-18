#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "damiao_motor_driver/motor_hw_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_hw_interface_node");
    ros::NodeHandle nh("~");

    pluginlib::ClassLoader<hardware_interface::RobotHW> loader(
        "damiao_motor_driver", "hardware_interface::RobotHW");

    boost::shared_ptr<hardware_interface::RobotHW> hw;
    try {
        hw = loader.createInstance("damiao_motor_driver/MotorHWInterface");
    }
    catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Failed to create MotorHWInterface: " << ex.what());
        return 1;
    }

    auto motor_hw = boost::dynamic_pointer_cast<MotorHWInterface>(hw);
    if (!motor_hw) {
        ROS_FATAL("Loaded hardware interface does not match MotorHWInterface type");
        return 1;
    }

    controller_manager::ControllerManager cm(motor_hw.get(), nh);

    double loop_hz = 500.0;
    nh.param("loop_hz", loop_hz, loop_hz);
    ros::Rate rate(loop_hz);

    ros::Time last_time = ros::Time::now();
    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const ros::Duration period = now - last_time;
        last_time = now;

        motor_hw->read();
        cm.update(now, period);
        motor_hw->write();

        rate.sleep();
    }

    return 0;
}
