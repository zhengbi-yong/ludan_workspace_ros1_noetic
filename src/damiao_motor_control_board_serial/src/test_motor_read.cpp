#include "ros/ros.h"
#include <damiao_motor_control_board_serial/motors_control_board_connect.h>
#include <vector>
#include <iostream>

/**
 * @brief 电机反馈读取程序
 * 
 * 功能：
 *  - 初始化 ROS 节点
 *  - 创建串口连接（Motors 类）
 *  - 循环读取各个电机的反馈信息（位置、速度、力矩）
 *  - 定期打印到控制台
 * 
 * 注意：
 *  - 不会下发任何控制指令
 *  - 仅用于验证通信和反馈是否正常
 */

static constexpr int N = NUM_MOTORS;  // 14 电机

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_motor_feedback");
  ros::NodeHandle nh("~");
  ros::Rate r(100);  // 100Hz 读取

  // 创建 Motors 通信对象（自动初始化串口）
  damiao_motor_control_board_serial::Motors motors;

  ROS_INFO("read_motor_feedback started. Waiting for motor feedback...");

  // 用于保存反馈数据
  double pos, vel, tor;

  ros::Time last_print = ros::Time::now();

  while (ros::ok())
  {
    // 每隔 0.5s 打印一次所有电机数据
    if ((ros::Time::now() - last_print).toSec() > 0.5)
    {
      last_print = ros::Time::now();

      ROS_INFO("------------- Motor Feedback -------------");
      for (int i = 0; i < N; ++i)
      {
        motors.get_motor_data(pos, vel, tor, i);
        ROS_INFO("Motor[%02d] pos=%7.3f  vel=%7.3f  tor=%7.3f", i, pos, vel, tor);
      }
      ROS_INFO("------------------------------------------");
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
