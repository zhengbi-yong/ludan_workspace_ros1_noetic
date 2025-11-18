#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "motor_serial.h"
#include "motor_protocol.h"

// 引入 ROS 消息类型
#include <damiao_motor_control_board_serial/MotorStates.h>
#include <damiao_motor_control_board_serial/MotorState.h>

class MotorDriver
{
public:
    explicit MotorDriver(const ros::NodeHandle& nh, std::shared_ptr<MotorTransport> transport = nullptr);

    static constexpr size_t motor_count() { return MOTOR_COUNT; }

    enum class CommandStatus
    {
        Ok,
        Limited,
        SerialError,
    };

    void start();     // 启动反馈线程
    void stop();      // 停止反馈线程

    CommandStatus send_cmd(int id, float p, float v, float kp, float kd, float torque);
    bool is_running() const { return running_.load(); }
    void send_safe_mode_frame();
    std::vector<damiao_motor_control_board_serial::MotorState> get_states() const;
    ros::Time get_last_feedback_time() const { return last_feedback_time_; }

private:
    void feedback_loop();
    void publish_states();
    void publish_status();
    void load_motor_metadata();
    void populateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

    ros::NodeHandle private_nh_;
    ros::NodeHandle pub_nh_;
    std::string tf_prefix_;
    std::shared_ptr<MotorTransport> serial_;
    std::thread fb_thread_;
    std::atomic<bool> running_;

    ros::Publisher motor_pub_;
    ros::Publisher status_pub_;

    // ❗必须写完整命名空间（你的最重要错误）
    std::vector<damiao_motor_control_board_serial::MotorState> motors_;

    mutable std::mutex motors_mutex_;

    // Diagnostics and parsing state
    diagnostic_updater::Updater diag_updater_;
    size_t checksum_failures_;
    size_t dirty_bytes_dropped_;
    size_t parse_failure_streak_;
    size_t timeout_streak_;
    size_t reconnect_requests_;
    size_t frames_received_;
    size_t frames_lost_;
    size_t command_limited_count_;
    double feedback_period_avg_;
    ros::Time last_feedback_time_;

    static constexpr uint8_t FRAME_HEADER = 0x7B;
    static constexpr size_t MOTOR_DATA_SIZE = 5;
    static constexpr size_t MOTOR_COUNT = 30;
    static constexpr size_t FRAME_SIZE = 1 + MOTOR_COUNT * MOTOR_DATA_SIZE + 1; // 152
    static constexpr size_t FAILURE_RECONNECT_THRESHOLD = 5;
};
