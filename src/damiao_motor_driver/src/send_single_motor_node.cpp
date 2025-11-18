#include <ros/ros.h>
#include "damiao_motor_driver/motor_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_single_motor_node");
    ros::NodeHandle nh("~");

    int id;
    nh.param("id", id, 0);
    ROS_INFO("Control single motor id=%d", id);

    MotorDriver driver(nh);
    driver.start();                // *********** 必须加 ***********

    ros::Rate rate(100);

    while (ros::ok())
    {
        driver.send_cmd(id, 0.0, 0.0, 10.0, 1.0, 0.0);
        rate.sleep();
    }

    driver.stop();                 // *********** 建议加 ***********
    return 0;
}
