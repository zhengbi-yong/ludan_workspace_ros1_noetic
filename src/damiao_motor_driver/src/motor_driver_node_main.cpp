#include <ros/ros.h>
#include "damiao_motor_driver/motor_driver_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    damiao_motor_driver::MotorDriverNode node(nh, private_nh);
    
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize motor driver node");
        return 1;
    }
    
    node.run();
    node.shutdown();
    
    return 0;
}

