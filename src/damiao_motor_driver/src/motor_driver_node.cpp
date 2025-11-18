#include <ros/ros.h>
#include "damiao_motor_driver/motor_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver_node");
    ros::NodeHandle nh;

    MotorDriver driver(nh);
    driver.start();

    ros::Rate rate(50);
    while (ros::ok() && driver.is_running()) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("motor_driver_node stopping, entering safe mode");
    driver.send_safe_mode_frame();
    driver.stop();
    return 0;
}
