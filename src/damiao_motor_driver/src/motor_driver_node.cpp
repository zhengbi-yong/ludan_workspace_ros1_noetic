#include <ros/ros.h>
#include "damiao_motor_driver/motor_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver_node");
    ros::NodeHandle nh;

    MotorDriver driver(nh);
    driver.start();

    ros::spin();

    driver.stop();
    return 0;
}
