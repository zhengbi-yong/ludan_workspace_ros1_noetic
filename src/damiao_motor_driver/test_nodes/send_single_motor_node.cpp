#include <ros/ros.h>
#include <damiao_motor_driver/SendCommand.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "send_single_motor_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    int id;
    private_nh.param("id", id, 0);
    ROS_INFO("Control single motor id=%d", id);

    // Wait for motor_driver_node service
    std::string service_name = private_nh.param<std::string>("service_name", "motor_driver_node/send_command");
    ROS_INFO("Waiting for service: %s", service_name.c_str());
    
    if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
        ROS_ERROR("Service %s not available. Make sure motor_driver_node is running.", service_name.c_str());
        return 1;
    }

    ros::ServiceClient client = nh.serviceClient<damiao_motor_driver::SendCommand>(service_name);

    ros::Rate rate(100);

    while (ros::ok())
    {
        damiao_motor_driver::SendCommand srv;
        srv.request.id = id;
        srv.request.p = 0.0;
        srv.request.v = 0.0;
        srv.request.kp = 10.0;
        srv.request.kd = 1.0;
        srv.request.torque = 0.0;

        if (client.call(srv)) {
            if (!srv.response.success) {
                ROS_WARN_THROTTLE(1.0, "Command failed: %s", srv.response.message.c_str());
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "Failed to call service");
        }

        rate.sleep();
    }

    ROS_INFO("Stopping single motor node");
    return 0;
}

