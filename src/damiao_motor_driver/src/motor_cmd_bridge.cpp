#include <ros/ros.h>
#include <damiao_motor_driver/MotorCommand.h>
#include <damiao_motor_driver/SendCommand.h>

ros::ServiceClient g_send_command_client;

void cmdCallback(const damiao_motor_driver::MotorCommand::ConstPtr& msg)
{
    if (!g_send_command_client.exists()) {
        ROS_WARN_THROTTLE(1.0, "SendCommand service not available, ignore cmd");
        return;
    }

    damiao_motor_driver::SendCommand srv;
    srv.request.id = msg->id;
    srv.request.p = msg->p;
    srv.request.v = msg->v;
    srv.request.kp = msg->kp;
    srv.request.kd = msg->kd;
    srv.request.torque = msg->torque;

    if (!g_send_command_client.call(srv)) {
        ROS_WARN_THROTTLE(1.0, "Failed to call SendCommand service for motor %d", msg->id);
        return;
    }

    if (!srv.response.success) {
        ROS_WARN_THROTTLE(1.0, "SendCommand failed for motor %d: %s", msg->id, srv.response.message.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "damiao_motor_cmd_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Wait for motor_driver_node service
    std::string service_name = private_nh.param<std::string>("service_name", "motor_driver_node/send_command");
    ROS_INFO("Waiting for service: %s", service_name.c_str());
    
    if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
        ROS_ERROR("Service %s not available. Make sure motor_driver_node is running.", service_name.c_str());
        return 1;
    }

    g_send_command_client = nh.serviceClient<damiao_motor_driver::SendCommand>(service_name);

    // Subscribe to control topic /motor_cmd
    ros::Subscriber sub = nh.subscribe("motor_cmd", 10, cmdCallback);

    ROS_INFO("Motor command bridge started, listening to /motor_cmd topic");
    ros::spin();

    return 0;
}
