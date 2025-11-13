#ifndef DMBOT_SERIAL_ROBOT_CONNECT_H
#define DMBOT_SERIAL_ROBOT_CONNECT_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include <thread>
#include <vector>
#include <string>
#include <cstdint>
#include <mutex>
#include "dmbot_serial/motor_data.h"

namespace dmbot_serial
{

class robot
{
public:
  robot();
  ~robot();

  void init_motor_serial();
  void get_motor_data_thread();
  void pub_joint_states();
  void send_motor_data();
  void fresh_cmd_motor_data(double pos, double vel, double torque, double kp, double kd, int motor_idx);
  void get_motor_data(double &pos, double &vel, double &torque, int motor_idx);
  unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);

  void dm4310_fbdata(motor_data_t& moto, uint8_t *data);
  void dm4340_fbdata(motor_data_t& moto, uint8_t *data);
  void dm6006_fbdata(motor_data_t& moto, uint8_t *data);
  void dm8006_fbdata(motor_data_t& moto, uint8_t *data);
  void dm6248p_fbdata(motor_data_t& moto, uint8_t *data);
  void dm10010l_fbdata(motor_data_t& moto, uint8_t *data);

  int16_t float_to_uint(float x_float, float x_min, float x_max, int bits);
  float uint_to_float(int x_int, float x_min, float x_max, int bits);

private:
  ros::NodeHandle n;
  ros::Publisher joint_state_pub;
  serial::Serial serial_motor;
  std::thread rec_thread;
  bool stop_thread_ = false;

  std::string motor_serial_port;
  int motor_seial_baud;

  // ✅ 以下是关键新增
  std::vector<motor_data_t> motors;    // 存储所有电机数据
  motor_comm Receive_Data;             // 接收缓存
  motor_comm Send_Data;                // 发送缓存
};

} // namespace dmbot_serial

#endif
