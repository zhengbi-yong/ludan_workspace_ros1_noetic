#include <ros/ros.h>
#include <cmath>
#include <damiao_motor_driver/SendCommands.h>
#include <damiao_motor_driver/MotorCommand.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Wait for motor_driver_node service
    std::string service_name = private_nh.param<std::string>("service_name", "motor_driver_node/send_commands");
    ROS_INFO("Waiting for service: %s", service_name.c_str());
    
    if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
        ROS_ERROR("Service %s not available. Make sure motor_driver_node is running.", service_name.c_str());
        return 1;
    }

    ros::ServiceClient client = nh.serviceClient<damiao_motor_driver::SendCommands>(service_name);

    ros::Rate rate(500);

    double t = 0;

    while (ros::ok())
    {
        double pos = 0.5 * sin(2 * M_PI * 0.5 * t);
        double vel = 0.5 * 2 * M_PI * 0.5 * cos(2 * M_PI * 0.5 * t);

        damiao_motor_driver::SendCommands srv;
        srv.request.commands.resize(30);
        for (int id = 0; id < 30; id++)
        {
            srv.request.commands[id].id = id;
            srv.request.commands[id].p = pos;
            srv.request.commands[id].v = vel;
            srv.request.commands[id].kp = 30.0;
            srv.request.commands[id].kd = 1.0;
            srv.request.commands[id].torque = 0.0;
        }

        if (client.call(srv)) {
            if (!srv.response.success) {
                ROS_WARN_THROTTLE(1.0, "Commands failed: %s", srv.response.message.c_str());
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "Failed to call service");
        }

        ROS_INFO_THROTTLE(0.3, "Trajectory pos=%.3f vel=%.3f", pos, vel);

        t += 0.002;
        rate.sleep();
    }

    ROS_INFO("Trajectory controller stopping");
    return 0;
}

