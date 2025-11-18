#include <damiao_motor_driver/motor_driver.h>
#include <iostream>

MotorDriver::MotorDriver(ros::NodeHandle& nh)
    : private_nh_("~"), serial_(private_nh_), running_(false)
{
    ROS_INFO("MotorDriver constructor entered !!!");
    motors_.resize(30);
    motor_pub_ = nh.advertise<damiao_motor_control_board_serial::MotorStates>("motor_states", 10);
}

void MotorDriver::start()
{
    if (!serial_.open()) {
        ROS_ERROR("Failed to open motor serial port after retries!");
        return;
    }

    running_ = true;
    fb_thread_ = std::thread(&MotorDriver::feedback_loop, this);
}

void MotorDriver::stop()
{
    running_ = false;
    if (fb_thread_.joinable())
        fb_thread_.join();

    serial_.close();
}

void MotorDriver::send_cmd(int id, float p, float v, float kp, float kd, float torque)
{
    uint8_t tx[11];
    MITProtocol::encode_cmd(tx, id, p, v, kp, kd, torque);
    MotorSerial::IOResult write_result = serial_.write_bytes(tx, 11);
    if (write_result.error != MotorSerial::IOError::None) {
        ROS_WARN_THROTTLE(2.0, "Failed to send command to motor %d (error %d)", id,
                          static_cast<int>(write_result.error));
    }
}

void MotorDriver::feedback_loop()
{
    ROS_INFO("feedback_loop started");

    uint8_t rx[152];

    while (running_ && ros::ok()) {
        MotorSerial::IOResult result = serial_.read_bytes(rx, sizeof(rx));

        if (result.error == MotorSerial::IOError::Timeout) {
            continue;
        }

        if (result.error != MotorSerial::IOError::None) {
            ROS_WARN_THROTTLE(2.0, "Serial read error (code %d)", static_cast<int>(result.error));
            continue;
        }

        if (result.bytes != sizeof(rx)) {
            ROS_WARN_THROTTLE(2.0, "Unexpected motor feedback size: %zu", result.bytes);
            continue;
        }

        for (int i = 0; i < 30; i++) {

            uint8_t* p = &rx[1 + i * 5];

            int p_int = (p[0] << 8) | p[1];
            int v_int = (p[2] << 4) | (p[3] >> 4);
            int t_int = ((p[3] & 0x0F) << 8) | p[4];

            motors_[i].pos = MITProtocol::uint_to_float(p_int, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
            motors_[i].vel = MITProtocol::uint_to_float(v_int, MITProtocol::V_MIN, MITProtocol::V_MAX, 12);
            motors_[i].tor = MITProtocol::uint_to_float(t_int, MITProtocol::T_MIN, MITProtocol::T_MAX, 12);
        }

        publish_states();
    }
}

void MotorDriver::publish_states()
{
    damiao_motor_control_board_serial::MotorStates msg;
    msg.header.stamp = ros::Time::now();
    msg.motors.resize(30);

    for (int i = 0; i < 30; i++) {
        msg.motors[i].id = i;
        msg.motors[i].pos = motors_[i].pos;
        msg.motors[i].vel = motors_[i].vel;
        msg.motors[i].tor = motors_[i].tor;
    }

    motor_pub_.publish(msg);
}
