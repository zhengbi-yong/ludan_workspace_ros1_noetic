#include <ros/ros.h>
#include <cmath>
#include <damiao_motor_driver/motor_driver.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_controller");
    ros::NodeHandle nh;

    MotorDriver driver(nh);
    driver.start();

    ros::Rate rate(500);

    double t = 0;

    while (ros::ok())
    {
        double pos = 0.5 * sin(2 * M_PI * 0.5 * t);
        double vel = 0.5 * 2 * M_PI * 0.5 * cos(2 * M_PI * 0.5 * t);

        for (int id = 0; id < 30; id++)
        {
            driver.send_cmd(id, pos, vel, 30, 1, 0);
        }

        ROS_INFO_THROTTLE(0.3, "Trajectory pos=%.3f vel=%.3f", pos, vel);

        t += 0.002;
        rate.sleep();
    }

    return 0;
}
