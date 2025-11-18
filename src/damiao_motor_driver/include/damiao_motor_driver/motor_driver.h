#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include <ros/ros.h>

#include "motor_serial.h"
#include "motor_protocol.h"

// 引入 ROS 消息类型
#include <damiao_motor_control_board_serial/MotorStates.h>
#include <damiao_motor_control_board_serial/MotorState.h>

class MotorDriver
{
public:
    explicit MotorDriver(ros::NodeHandle& nh);

    void start();     // 启动反馈线程
    void stop();      // 停止反馈线程

    void send_cmd(int id, float p, float v, float kp, float kd, float torque);
        const std::vector<damiao_motor_control_board_serial::MotorState>& get_states() const {
        return motors_;
    }

private:
    void feedback_loop();
    void publish_states();

    MotorSerial serial_;
    std::thread fb_thread_;
    std::atomic<bool> running_;

    ros::Publisher motor_pub_;

    // ❗必须写完整命名空间（你的最重要错误）
    std::vector<damiao_motor_control_board_serial::MotorState> motors_;
};
