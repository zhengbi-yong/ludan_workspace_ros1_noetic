#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <damiao_motor_control_board_serial/motors_control_board_connect.h>
#include <damiao_motor_control_board_serial/motor_data.h>
#include <damiao_motor_control_board_serial/MotorStates.h>
#include <algorithm>
#include <cmath>


namespace damiao_motor_control_board_serial
{
Motors::Motors()
{
  _node_handle.param("port", _port, std::string("/dev/ttyACM0")); 
  _node_handle.param("baud", _baud, 921600);
  motors.resize(NUM_MOTORS);
  /*初始化电机的参数，此处应改成配置文件驱动。*/
  int i=0;
  for(auto& motor:motors)
  {
      motor.name = "motor_" + std::to_string(i);   
      motor.pos = 0.0f;
      motor.vel = 0.0f;
      motor.tor = 0.0f;
      motor.tor_set = 0.0f;
      motor.pos_set = 0.0f;
      motor.vel_set = 0.0f;
      motor.kp = 0.0f;
      motor.kd = 0.0f;
      motor.index=i;
      i++;
  }
  /*此处给电机设置类型，硬编码，需要更改。*/
  motors[0].type = "10010l";
  motors[1].type = "10010l";
  motors[2].type = "10010l";
  motors[3].type = "6248p";                
  motors[4].type = "4340";
  motors[5].type = "4340";   
  motors[6].type = "4340";
    
  motors[7].type = "6248p";
  motors[8].type = "6248p";
  motors[9].type = "6248p";                
  motors[10].type = "4340";
  motors[11].type = "4340";
  motors[12].type = "4340";                
  motors[13].type = "4340";

  init_serial_port();//初始化串口
  /*接受电机数据的线程*/
  rec_thread = std::thread(&Motors::get_motor_states_thread, this);
  /*发布电机数据*/
  _joint_states_publisher = _node_handle.advertise<sensor_msgs::JointState>("joint_states", 10);   
  /*等待*/
  ros::Duration(2.0).sleep();

  ROS_INFO("Motors init complete");

}

Motors::~Motors()
{
    std::string node_name = ros::this_node::getName();

    ROS_INFO_STREAM("[" << node_name << "] Shutting down...");

    try 
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            fresh_cmd_motor_data(0.0, 0.0, 0.0, 0.0, 0.0, i);
        }

        send_motor_data();
        ROS_INFO_STREAM("[" << node_name << "] Motor stop commands sent successfully.");

        _is_thread_stopped = true;

        if (rec_thread.joinable())
        {
            ROS_INFO_STREAM("[" << node_name << "] Waiting for feedback thread to stop...");
            rec_thread.join();
            ROS_INFO_STREAM("[" << node_name << "] Feedback thread stopped.");
        }

        if (_serial_port.isOpen())
        {
            _serial_port.close();
            ROS_INFO_STREAM("[" << node_name << "] Serial port closed.");
        }

        ROS_INFO_STREAM("[" << node_name << "] Shutdown complete.");

    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("[" << node_name << "] Exception during shutdown: " << e.what());
    }
    catch (...)
    {
        ROS_ERROR_STREAM("[" << node_name << "] Unknown error during shutdown.");
    }
}


void Motors::init_serial_port()
{
    try
    {
        std::string node_name = ros::this_node::getName();

        _serial_port.setPort(_port);
        _serial_port.setBaudrate(_baud);
        _serial_port.setFlowcontrol(serial::flowcontrol_none);
        _serial_port.setParity(serial::parity_none);     // 无奇偶校验
        _serial_port.setStopbits(serial::stopbits_one);   // 1个停止位
        _serial_port.setBytesize(serial::eightbits);      // 8个数据位

        serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
        _serial_port.setTimeout(time_out);

        _serial_port.open();

        if (!_serial_port.isOpen())
        {
            throw std::runtime_error("Serial port opened but not active.");
        }

        ROS_INFO_STREAM("[" << node_name << "] Motor serial port initialized successfully: "
                            << _port << " @ " << _baud << " baud.");
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR_STREAM("[" << ros::this_node::getName()
                             << "] IOException: failed to open serial port '"
                             << _port << "' (" << e.what() << ")");
        ros::shutdown();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("[" << ros::this_node::getName()
                             << "] Exception: " << e.what());
        ros::shutdown();
    }
    catch (...)
    {
        ROS_ERROR_STREAM("[" << ros::this_node::getName()
                             << "] Unknown error occurred while initializing serial port.");
        ros::shutdown();
    }
}

void Motors::get_motor_states_thread()
{
  std::string node_name = ros::this_node::getName();
  ROS_INFO_STREAM("[" << node_name << "] Feedback thread started.");

  ros::Rate rate(200);  // 主循环频率（Hz）
  std::vector<uint8_t> rx_buf(RECEIVE_DATA_SIZE);
  ros::Time last_feedback_time = ros::Time::now();

  while (ros::ok() && !_is_thread_stopped)
  {
    try
    {
      // 1️⃣ 检查串口是否打开
      if (!_serial_port.isOpen())
      {
        ROS_WARN_THROTTLE(1.0, "[%s] Serial port closed, trying to reopen...", node_name.c_str());
        try {
          _serial_port.close();
          init_serial_port();  // 尝试重新初始化
          ros::Duration(0.2).sleep();
          continue;
        } catch (...) {
          ros::Duration(0.5).sleep();
          continue;
        }
      }

      // 2️⃣ 读取数据帧
      size_t n = _serial_port.read(rx_buf.data(), RECEIVE_DATA_SIZE);
      if (n != RECEIVE_DATA_SIZE)
      {
        rate.sleep();
        continue;
      }

      // 3️⃣ 帧头检查
      if (rx_buf[0] != FRAME_HEADER)
      {
        ROS_WARN_THROTTLE(1.0, "[%s] Invalid frame header (0x%02X).", node_name.c_str(), rx_buf[0]);
        continue;
      }

      // 4️⃣ 校验 XOR 校验码
      uint8_t xor_check = 0;
      for (int i = 0; i < RECEIVE_DATA_SIZE - 1; ++i)
        xor_check ^= rx_buf[i];

      if (xor_check != rx_buf[RECEIVE_DATA_SIZE - 1])
      {
        ROS_WARN_THROTTLE(1.0, "[%s] Frame checksum mismatch.", node_name.c_str());
        continue;
      }

      // 5️⃣ 有效帧，复制到接收缓存
      memcpy(Receive_Data.rx, rx_buf.data(), RECEIVE_DATA_SIZE);

      // 6️⃣ 解析反馈数据（每电机 5 字节）
      for (int i = 0; i < NUM_MOTORS; ++i)
      {
        uint8_t *p = &Receive_Data.rx[1 + i * 5];
        auto &m = motors[i];

        if      (m.type == "4340")   dm4340_fbdata(m, p);
        else if (m.type == "4310")   dm4310_fbdata(m, p);
        else if (m.type == "6006")   dm6006_fbdata(m, p);
        else if (m.type == "8006")   dm8006_fbdata(m, p);
        else if (m.type == "6248p")  dm6248p_fbdata(m, p);
        else if (m.type == "10010l") dm10010l_fbdata(m, p);
      }

      // 7️⃣ 发布电机状态
      pub_motor_states();

      // 8️⃣ 更新反馈时间
      last_feedback_time = ros::Time::now();
    }
    catch (const serial::IOException &e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Serial IO exception: %s", node_name.c_str(), e.what());
      ros::Duration(0.1).sleep();
      continue;
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Exception in feedback thread: %s", node_name.c_str(), e.what());
      ros::Duration(0.1).sleep();
      continue;
    }
    catch (...)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s] Unknown exception in feedback thread.", node_name.c_str());
      ros::Duration(0.1).sleep();
      continue;
    }

    // 9️⃣ 检查反馈超时
    if ((ros::Time::now() - last_feedback_time).toSec() > 1.0)
    {
      ROS_WARN_THROTTLE(2.0, "[%s] No feedback for over 1s.", node_name.c_str());
    }

    rate.sleep();
  }

  ROS_WARN_STREAM("[" << node_name << "] Feedback thread stopped.");
}

void Motors::pub_motor_states()
{
  // 创建消息对象
  damiao_motor_control_board_serial::MotorStates msg;
  msg.header.stamp = ros::Time::now();
  msg.motors.resize(NUM_MOTORS);

  // 填充所有电机状态
  for (int i = 0; i < NUM_MOTORS; ++i)
  {
    msg.motors[i].id = motors[i].index;
    msg.motors[i].type = motors[i].type;
    msg.motors[i].pos = motors[i].pos;
    msg.motors[i].vel = motors[i].vel;
    msg.motors[i].tor = motors[i].tor;
    msg.motors[i].state = motors[i].state;
  }

  // 一次性发布
  _motor_states_publisher.publish(msg);
}


void Motors::pub_joint_states()
{
  ros::Rate rate(200); 
  while (ros::ok())
  {
      sensor_msgs::JointState joint_state_msg;

      for (int i = 0; i < NUM_MOTORS; ++i)
      {
          joint_state_msg.name.push_back(motors[i].name);

          joint_state_msg.position.push_back(motors[i].pos);
          ROS_INFO_STREAM_THROTTLE(0.5, "position[" << i << "] = "
                                            << joint_state_msg.position.back());

          joint_state_msg.velocity.push_back(motors[i].vel);
          joint_state_msg.effort.push_back(motors[i].tor);
      }

    _joint_states_publisher.publish(joint_state_msg);

    rate.sleep();
  }
}

void Motors::send_motor_data()
{ 
  // 新加的保护逻辑  
  bool all_feedback_ok = true;
  for (int i = 0; i < NUM_MOTORS; ++i)
  {
    // 检查是否已经有有效反馈（非NaN，且非0初始值）
    if (!std::isfinite(motors[i].pos) || fabs(motors[i].pos) < 1e-6)
    {
      all_feedback_ok = false;
      break;
    }
  }

  if (!all_feedback_ok)
  {
    ROS_WARN_THROTTLE(1.0, "[Motors_connect] Feedback not ready, skip sending commands to prevent jump.");
    return;  // 直接返回，不发数据
  }
  //原来的代码
  ros::Time last_time2 = ros::Time::now();
  for(auto& motor:motors)
  {  
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    if(motor.type == "8006")
    {   
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN4,  P_MAX4,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN4,  V_MAX4,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN4, KP_MAX4, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN4, KD_MAX4, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN4,  T_MAX4,  12);
    }
    else if(motor.type == "6006")
    {   
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN3,  P_MAX3,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN3,  V_MAX3,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN3, KP_MAX3, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN3, KD_MAX3, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN3,  T_MAX3,  12);
    }
    else  if(motor.type == "4340")
    {
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN2,  P_MAX2,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN2,  V_MAX2,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN2, KP_MAX2, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN2, KD_MAX2, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN2,  T_MAX2,  12);
    }
    else  if(motor.type == "4310")
    {   
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN1,  P_MAX1,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN1,  V_MAX1,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN1, KP_MAX1, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN1, KD_MAX1, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN1,  T_MAX1,  12);
    }
    else  if(motor.type == "6248p")
    {   
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN5,  P_MAX5,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN5,  V_MAX5,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN5, KP_MAX5, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN5, KD_MAX5, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN5,  T_MAX5,  12);
    }
    else  if(motor.type == "10010l")
    {   
        pos_tmp = float_to_uint(motor.pos_set,  P_MIN6,  P_MAX6,  16);
        vel_tmp = float_to_uint(motor.vel_set,  V_MIN6,  V_MAX6,  12);
        kp_tmp  = float_to_uint(motor.kp,   KP_MIN6, KP_MAX6, 12);
        kd_tmp  = float_to_uint(motor.kd,   KD_MIN6, KD_MAX6, 12);
        tor_tmp = float_to_uint(motor.tor_set, T_MIN6,  T_MAX6,  12);
    }
    Send_Data.tx[0]=FRAME_HEADER; 
    Send_Data.tx[1]=motor.index;

    Send_Data.tx[2] = (pos_tmp >> 8);
    Send_Data.tx[3] = pos_tmp;
    Send_Data.tx[4] = (vel_tmp >> 4);
    Send_Data.tx[5] = ((vel_tmp&0x0F)<<4)|(kp_tmp>>8);
    Send_Data.tx[6] = kp_tmp;
    Send_Data.tx[7] = (kd_tmp >> 4);
    Send_Data.tx[8] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    Send_Data.tx[9] = tor_tmp;

    Send_Data.tx[10]=check_sum(10,SEND_DATA_CHECK);   
     
    try
    { //通过串口向下位机发送数据 
      _serial_port.write(Send_Data.tx,sizeof(Send_Data.tx));
    //ROS_INFO("Current time Motor: %f", interval.toSec());
    }
    catch (serial::IOException& e)   
    {
      ROS_ERROR_STREAM("In send_motor_data,Unable to send data through motor serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
    }

  }  
        
}


void Motors::fresh_cmd_motor_data(double pos, double vel,double torque, double kp,double kd,int motor_idx)
{//更新发给电机的参数、力矩等
  motors[motor_idx].pos_set = pos;
  motors[motor_idx].vel_set = vel;
  motors[motor_idx].tor_set = torque;
  motors[motor_idx].kp = kp;
  motors[motor_idx].kd = kd;            
}

void Motors::get_motor_data(double &pos,double &vel,double &torque, int motor_idx) //Monitor TODO
{//获取电机反馈的参数、力矩等
  pos = motors[motor_idx].pos;
  // ROS_INFO_STREAM_THROTTLE(0.5, "get pos[" << motor_idx << "] = " << pos);
  vel = motors[motor_idx].vel;
  torque =motors[motor_idx].tor;
  // moto.id = (data[5]>>4)&0x0F;
  // moto.state = (data[5])&0x0F;

}

/**************************************
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char Motors::check_sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum=0,k;

    if(mode==0) //Receive data mode //接收数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
        }
    }

    if(mode==1) //Send data mode //发送数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
        }
    }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

void Motors::dm4310_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.id = (data[5]>>4)&0x0F;
  moto.state = (data[5])&0x0F;
  moto.pos = uint_to_float(moto.p_int, P_MIN1, P_MAX1, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN1, V_MAX1, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN1, T_MAX1, 12);  // (-10.0,10.0)
}

void Motors::dm4340_fbdata(motor_data_t& moto,uint8_t *data)   // Moniter TODO
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.id = (data[5]>>4)&0x0F;
  moto.state = (data[5])&0x0F;
  moto.pos = uint_to_float(moto.p_int, P_MIN2, P_MAX2, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN2, V_MAX2, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN2, T_MAX2, 12);  // (-10.0,10.0)
}

void Motors::dm6006_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.id = (data[5]>>4)&0x0F;
  moto.state = (data[5])&0x0F;
  moto.pos = uint_to_float(moto.p_int, P_MIN3, P_MAX3, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN3, V_MAX3, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN3, T_MAX3, 12);  // (-10.0,10.0)
}

void Motors::dm8006_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.id = (data[5]>>4)&0x0F;
  moto.state = (data[5])&0x0F;
  moto.pos = uint_to_float(moto.p_int, P_MIN4, P_MAX4, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN4, V_MAX4, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN4, T_MAX4, 12);  // (-10.0,10.0)
}

void Motors::dm6248p_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.id = (data[5]>>4)&0x0F;
  moto.state = (data[5])&0x0F;
  moto.pos = uint_to_float(moto.p_int, P_MIN5, P_MAX5, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN5, V_MAX5, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN5, T_MAX5, 12);  // (-10.0,10.0)
}

void Motors::dm10010l_fbdata(motor_data_t& moto,uint8_t *data)
{
  moto.p_int=(data[0]<<8)|data[1];
  moto.v_int=(data[2]<<4)|(data[3]>>4);
  moto.t_int=((data[3]&0x0F)<<8)|data[4];
  moto.id = (data[5]>>4)&0x0F;
  moto.state = (data[5])&0x0F;
  moto.pos = uint_to_float(moto.p_int, P_MIN6, P_MAX6, 16); // (-12.5,12.5)
  moto.vel = uint_to_float(moto.v_int, V_MIN6, V_MAX6, 12); // (-30.0,30.0)
  moto.tor = uint_to_float(moto.t_int, T_MIN6, T_MAX6, 12);  // (-10.0,10.0)
}

int16_t Motors::float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  if (bits <= 0)
  {
    return 0;
  }

  const float span = x_max - x_min;
  if (span <= 0.0f)
  {
    return 0;
  }

  const float clamped = std::max(std::min(x_float, x_max), x_min);
  const float max_raw = static_cast<float>((1 << bits) - 1);
  float normalized = (clamped - x_min) / span;
  if (normalized < 0.0f)
  {
    normalized = 0.0f;
  }
  if (normalized > 1.0f)
  {
    normalized = 1.0f;
  }

  int raw = static_cast<int>(std::lrint(static_cast<double>(normalized * max_raw)));
  if (raw < 0)
  {
    raw = 0;
  }
  const int max_int = static_cast<int>(max_raw);
  if (raw > max_int)
  {
    raw = max_int;
  }

  return static_cast<int16_t>(raw);
}

float Motors::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  if (bits <= 0)
  {
    return x_min;
  }

  const float span = x_max - x_min;
  if (span <= 0.0f)
  {
    return x_min;
  }

  const int mask = (1 << bits) - 1;
  x_int &= mask;
  const float max_raw = static_cast<float>(mask);
  return x_min + (static_cast<float>(x_int) / max_raw) * span;
}
}