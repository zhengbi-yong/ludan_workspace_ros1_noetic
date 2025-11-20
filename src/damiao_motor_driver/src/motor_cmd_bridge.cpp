#include <ros/ros.h>
#include <damiao_motor_driver/motor_driver.h>
#include <damiao_motor_driver/MotorCommand.h>

std::shared_ptr<MotorDriver> g_driver;

void cmdCallback(const damiao_motor_driver::MotorCommand::ConstPtr& msg)
{
    if (!g_driver || !g_driver->is_running()) {
        ROS_WARN_THROTTLE(1.0, "Motor driver not running, ignore cmd");
        return;
    }

    MotorDriver::CommandStatus status =
        g_driver->send_cmd(msg->id, msg->p, msg->v, msg->kp, msg->kd, msg->torque);

    switch (status) {
    case MotorDriver::CommandStatus::Ok:
        // 正常就不打日志，避免刷屏
        break;
    case MotorDriver::CommandStatus::Limited:
        ROS_WARN_THROTTLE(1.0, "Cmd for motor %d limited to protocol bounds", msg->id);
        break;
    case MotorDriver::CommandStatus::SerialError:
        ROS_WARN_THROTTLE(1.0, "Serial error while sending cmd to motor %d", msg->id);
        break;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "damiao_motor_cmd_bridge");
    ros::NodeHandle nh;

    // 创建并启动 MotorDriver：打开串口 + 启动反馈线程
    g_driver = std::make_shared<MotorDriver>(nh);
    g_driver->start();

    // 订阅控制话题 /motor_cmd
    ros::Subscriber sub = nh.subscribe("motor_cmd", 10, cmdCallback);

    ros::spin();

    // 退出时安全停
    g_driver->stop();
    return 0;
}
