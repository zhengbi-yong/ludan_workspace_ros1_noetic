#ifndef __MOTORS_CONTROL_BOARD_CONNECT_H
#define __MOTORS_CONTROL_BOARD_CONNECT_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include <thread>
#include <vector>
#include <string>
#include <cstdint>
#include <mutex>
#include <damiao_motor_control_board_serial/motor_data.h>
#include <damiao_motor_control_board_serial/MotorState.h>
#include <damiao_motor_control_board_serial/MotorStates.h>

namespace damiao_motor_control_board_serial
{

class Motors
{
public:
  Motors();
  ~Motors();

  void init_serial_port();
  void pub_motor_states();
  void get_motor_states_thread();
  void pub_joint_states();
  void send_motor_data();
  void fresh_cmd_motor_data(double pos, double vel, double torque, double kp, double kd, int motor_idx);
  void get_motor_data(double &pos, double &vel, double &torque, int motor_idx);
  unsigned char check_sum(unsigned char Count_Number, unsigned char mode);

  void dm4310_fbdata(motor_data_t& moto, uint8_t *data);
  void dm4340_fbdata(motor_data_t& moto, uint8_t *data);
  void dm6006_fbdata(motor_data_t& moto, uint8_t *data);
  void dm8006_fbdata(motor_data_t& moto, uint8_t *data);
  void dm6248p_fbdata(motor_data_t& moto, uint8_t *data);
  void dm10010l_fbdata(motor_data_t& moto, uint8_t *data);

  int16_t float_to_uint(float x_float, float x_min, float x_max, int bits);
  float uint_to_float(int x_int, float x_min, float x_max, int bits);
  

private:
  ros::NodeHandle _node_handle;
  ros::Publisher _motor_states_publisher;
  ros::Publisher _joint_states_publisher;

  serial::Serial _serial_port;
  
  bool _is_thread_stopped = false;

  std::thread rec_thread;
  std::vector<motor_data_t> motors;

  std::string _port;
  int _baud;

  motor_comm Receive_Data;
  motor_comm Send_Data;
};

} // namespace damiao_motor_control_board_serial

#endif
