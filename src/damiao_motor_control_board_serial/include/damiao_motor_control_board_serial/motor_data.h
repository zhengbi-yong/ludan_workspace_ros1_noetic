#ifndef DMBOT_SERIAL_MOTOR_DATA_H
#define DMBOT_SERIAL_MOTOR_DATA_H

#include <cstdint>
#include <string>

#define NUM_MOTORS 30
#define FRAME_HEADER 0x7B
#define READ_DATA_CHECK 0
#define SEND_DATA_CHECK 1
#define RECEIVE_DATA_SIZE 152
#define SEND_DATA_SIZE 11

// 电机范围定义
#define P_MIN1 -12.5f
#define P_MAX1  12.5f
#define V_MIN1 -30.0f
#define V_MAX1  30.0f
#define KP_MIN1 0.0f
#define KP_MAX1 500.0f
#define KD_MIN1 0.0f
#define KD_MAX1 5.0f
#define T_MIN1 -10.0f
#define T_MAX1  10.0f

// 其他型号可相同复制
#define P_MIN2 P_MIN1
#define P_MAX2 P_MAX1
#define V_MIN2 V_MIN1
#define V_MAX2 V_MAX1
#define KP_MIN2 KP_MIN1
#define KP_MAX2 KP_MAX1
#define KD_MIN2 KD_MIN1
#define KD_MAX2 KD_MAX1
#define T_MIN2 T_MIN1
#define T_MAX2 T_MAX1

#define P_MIN3 P_MIN1
#define P_MAX3 P_MAX1
#define V_MIN3 V_MIN1
#define V_MAX3 V_MAX1
#define KP_MIN3 KP_MIN1
#define KP_MAX3 KP_MAX1
#define KD_MIN3 KD_MIN1
#define KD_MAX3 KD_MAX1
#define T_MIN3 T_MIN1
#define T_MAX3 T_MAX1

#define P_MIN4 P_MIN1
#define P_MAX4 P_MAX1
#define V_MIN4 V_MIN1
#define V_MAX4 V_MAX1
#define KP_MIN4 KP_MIN1
#define KP_MAX4 KP_MAX1
#define KD_MIN4 KD_MIN1
#define KD_MAX4 KD_MAX1
#define T_MIN4 T_MIN1
#define T_MAX4 T_MAX1

#define P_MIN5 P_MIN1
#define P_MAX5 P_MAX1
#define V_MIN5 V_MIN1
#define V_MAX5 V_MAX1
#define KP_MIN5 KP_MIN1
#define KP_MAX5 KP_MAX1
#define KD_MIN5 KD_MIN1
#define KD_MAX5 KD_MAX1
#define T_MIN5 T_MIN1
#define T_MAX5 T_MAX1

#define P_MIN6 P_MIN1
#define P_MAX6 P_MAX1
#define V_MIN6 V_MIN1
#define V_MAX6 V_MAX1
#define KP_MIN6 KP_MIN1
#define KP_MAX6 KP_MAX1
#define KD_MIN6 KD_MIN1
#define KD_MAX6 KD_MAX1
#define T_MIN6 T_MIN1
#define T_MAX6 T_MAX1

typedef struct
{
  uint8_t rx[RECEIVE_DATA_SIZE];
  uint8_t tx[SEND_DATA_SIZE];
} motor_comm;

typedef struct
{
  std::string name;
  std::string type;
  int index;
  int id;
  int state;
  float pos;
  float vel;
  float tor;
  float pos_set;
  float vel_set;
  float tor_set;
  float kp;
  float kd;

  int p_int;
  int v_int;
  int t_int;
} motor_data_t;

#endif
