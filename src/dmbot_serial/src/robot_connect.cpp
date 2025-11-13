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
#include <dmbot_serial/robot_connect.h>
#include <dmbot_serial/motor_data.h>
#include <algorithm>
#include <cmath>


namespace dmbot_serial
{
robot::robot()
{

  n.param("port", motor_serial_port, std::string("/dev/ttyACM0")); 
  n.param("baud", motor_seial_baud, 921600);
  motors.resize(NUM_MOTORS);
  int i=0;
  for(auto& motor:motors)
  {
      motor.name = "Motor_" + std::to_string(i);   
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
   
  init_motor_serial();//初始化串口

  rec_thread = std::thread(&robot::get_motor_data_thread, this);
  
  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);   

  ros::Duration(2.0).sleep();

  ROS_INFO("robot init complete");

}
robot::~robot()
{   
   for(int i=0;i<NUM_MOTORS;i++)
  {
    fresh_cmd_motor_data(0.0, 0.0, 0.0, 0.0, 0.0, i); //更新发给电机的参数、力矩等
  }
  
  send_motor_data();

  stop_thread_ = true;

  if(rec_thread.joinable())
  {
    rec_thread.join(); 
  }
  if (serial_motor.isOpen())
  {
    serial_motor.close(); 
  }
}

void robot::init_motor_serial()
{         
    try
    {
      serial_motor.setPort(motor_serial_port);
      serial_motor.setBaudrate(motor_seial_baud);
      serial_motor.setFlowcontrol(serial::flowcontrol_none);
      serial_motor.setParity(serial::parity_none); //default is parity_none
      serial_motor.setStopbits(serial::stopbits_one);
      serial_motor.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
      serial_motor.setTimeout(time_out);
      serial_motor.open();
    } 
    catch (serial::IOException &e)  // 抓取异常
    {
        ROS_ERROR_STREAM("In single initialization,Unable to open motor serial port ");
        exit(0);
    }
    if (serial_motor.isOpen())
    {
        ROS_INFO_STREAM("In single initialization,Motor Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM("In single initialization,Unable to open motor serial port ");
        exit(0);
    }
   
}


void robot::get_motor_data_thread()
{
  std::string ns = ros::this_node::getNamespace();  // 取命名空间

  // === 自动生成带命名空间的日志文件 ===
  time_t now = time(nullptr);
  char filename[256];
  snprintf(filename, sizeof(filename),
           "/home/ludan/ludan_ws/motor_logs/%s_motor_feedback_%s.log",
           ns.empty() ? "global" : ns.substr(1).c_str(),     // 去掉开头的'/'
           []{
             char buf[64];
             time_t t = time(nullptr);
             strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", localtime(&t));
             return std::string(buf);
           }().c_str());

  std::ofstream log_file(filename, std::ios::out);
  if (!log_file.is_open()) {
    ROS_ERROR("无法创建 motor_feedback.log 文件（路径可能不存在）！");
    return;
  }

  ROS_INFO("[robot_connect] Feedback thread started in namespace [%s], logging to %s",
           ns.c_str(), filename);

  ros::Time last_feedback_time = ros::Time::now();
  ros::Time last_log_time = ros::Time::now();
  ros::Time last_alive_log = ros::Time::now();
  std::vector<bool> first_feedback_received(NUM_MOTORS, false);

  uint8_t buffer[RECEIVE_DATA_SIZE];  // 完整帧缓存

  while (ros::ok() && !stop_thread_) {
    if (!serial_motor.isOpen()) {
      ROS_ERROR_THROTTLE(1.0, "[robot_connect] Serial port closed, trying reopen...");
      try { serial_motor.close(); init_motor_serial(); }
      catch (...) { ros::Duration(0.5).sleep(); continue; }
    }

    try {
      // ✅ 一次性读取完整帧（防止错位）
      size_t n = serial_motor.read(buffer, RECEIVE_DATA_SIZE);
      if (n != RECEIVE_DATA_SIZE) continue;

      // 校验帧头
      if (buffer[0] != FRAME_HEADER) continue;

      // 校验 XOR 校验码
      uint8_t check = 0;
      for (int k = 0; k < RECEIVE_DATA_SIZE - 1; k++) check ^= buffer[k];
      if (check != buffer[RECEIVE_DATA_SIZE - 1]) {
        ROS_WARN_THROTTLE(1.0, "[robot_connect] Frame checksum error, resyncing...");
        continue;
      }

      // ✅ 有效帧，复制到接收结构
      memcpy(Receive_Data.rx, buffer, RECEIVE_DATA_SIZE);

      // === 解析每个电机的数据 ===
      for (int i = 0; i < NUM_MOTORS; ++i) {
        uint8_t *p = &Receive_Data.rx[1 + i * 5];   // 每电机 5 字节反馈
        auto &m = motors[i];
        if      (m.type == "4340")   dm4340_fbdata(m, p);
        else if (m.type == "4310")   dm4310_fbdata(m, p);
        else if (m.type == "6006")   dm6006_fbdata(m, p);
        else if (m.type == "8006")   dm8006_fbdata(m, p);
        else if (m.type == "6248p")  dm6248p_fbdata(m, p);
        else if (m.type == "10010l") dm10010l_fbdata(m, p);

        if (!first_feedback_received[i]) {
          first_feedback_received[i] = true;
          ROS_INFO("[robot_connect] Motor %d first feedback received.", i);
        }
      }

      // --- 限频写入日志（1Hz）---
      if ((ros::Time::now() - last_log_time).toSec() > 1.0) {
        last_log_time = ros::Time::now();

        log_file << "Namespace: " << ns << std::endl;

        // 获取系统时间（人类可读格式）
        auto t = std::chrono::system_clock::now();
        std::time_t t_c = std::chrono::system_clock::to_time_t(t);
        std::tm local_tm = *std::localtime(&t_c);

        // 格式化时间到字符串
        char time_str[64];
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &local_tm);

        // 加上毫秒
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      t.time_since_epoch()) % 1000;

        // 写入日志
        log_file << "Timestamp: " << time_str << "." << std::setfill('0')
                 << std::setw(3) << ms.count() << std::endl;

        // 仅记录电机7~13数据
        for (int i = 7; i <= 13 && i < NUM_MOTORS; ++i) {
          log_file << "Motor[" << i << "] "
                   << "id=" << motors[i].id << "  "
                   << "state=" << motors[i].state << "  "
                   << "tor=" << motors[i].tor << std::endl;
        }

        log_file << "--------------------------------------" << std::endl;
        log_file.flush();
      }

      last_feedback_time = ros::Time::now();

    } catch (serial::IOException &e) {
      ROS_ERROR_THROTTLE(1.0, "[robot_connect] Serial read error: %s", e.what());
      continue;
    }

    // 定期打印线程心跳
    if ((ros::Time::now() - last_alive_log).toSec() > 2.0) {
      ROS_INFO("[robot_connect] [%s] thread alive, last feedback %.3fs ago",
               ns.c_str(), (ros::Time::now() - last_feedback_time).toSec());
      last_alive_log = ros::Time::now();
    }
  }

  log_file.close();
  ROS_WARN("[robot_connect] [%s] Feedback thread stopped, log closed.", ns.c_str());
}

void robot::pub_joint_states()
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

    joint_state_pub.publish(joint_state_msg);

    rate.sleep();
  }
}

void robot::send_motor_data()
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
    ROS_WARN_THROTTLE(1.0, "[robot_connect] Feedback not ready, skip sending commands to prevent jump.");
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
      serial_motor.write(Send_Data.tx,sizeof(Send_Data.tx));
    //ROS_INFO("Current time Motor: %f", interval.toSec());
    }
    catch (serial::IOException& e)   
    {
      ROS_ERROR_STREAM("In send_motor_data,Unable to send data through motor serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
    }

  }  
        
}


void robot::fresh_cmd_motor_data(double pos, double vel,double torque, double kp,double kd,int motor_idx)
{//更新发给电机的参数、力矩等
  motors[motor_idx].pos_set = pos;
  motors[motor_idx].vel_set = vel;
  motors[motor_idx].tor_set = torque;
  motors[motor_idx].kp = kp;
  motors[motor_idx].kd = kd;            
}

void robot::get_motor_data(double &pos,double &vel,double &torque, int motor_idx) //Monitor TODO
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
unsigned char robot::check_sum(unsigned char Count_Number,unsigned char mode)
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

void robot::dm4310_fbdata(motor_data_t& moto,uint8_t *data)
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

void robot::dm4340_fbdata(motor_data_t& moto,uint8_t *data)   // Moniter TODO
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

void robot::dm6006_fbdata(motor_data_t& moto,uint8_t *data)
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

void robot::dm8006_fbdata(motor_data_t& moto,uint8_t *data)
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

void robot::dm6248p_fbdata(motor_data_t& moto,uint8_t *data)
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

void robot::dm10010l_fbdata(motor_data_t& moto,uint8_t *data)
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

int16_t robot::float_to_uint(float x_float, float x_min, float x_max, int bits)
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

float robot::uint_to_float(int x_int, float x_min, float x_max, int bits)
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