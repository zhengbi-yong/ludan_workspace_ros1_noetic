#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <serial/serial.h>

class MotorSerial {
public:
    enum class IOError {
        None = 0,
        NotOpen,
        Timeout,
        IOError,
    };

    struct IOResult {
        size_t bytes = 0;
        IOError error = IOError::None;
    };

    explicit MotorSerial(ros::NodeHandle& nh);
    ~MotorSerial();

    bool open();
    void close();
    bool is_open();

    IOResult write_bytes(const uint8_t* data, size_t size);
    IOResult read_bytes(uint8_t* buffer, size_t size);

private:
    bool open_once();
    bool ensure_connection_locked(std::unique_lock<std::mutex>& lock);
    bool reconnect_locked(std::unique_lock<std::mutex>& lock);
    void load_parameters(ros::NodeHandle& nh);
    void populateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void update_timeouts_locked();

    serial::Serial ser_;
    std::string port_;
    int baud_;
    uint32_t read_timeout_ms_;
    uint32_t write_timeout_ms_;
    ros::Duration heartbeat_timeout_;
    ros::Duration reconnect_backoff_;
    ros::Duration reconnect_backoff_max_;
    int max_reconnect_attempts_;

    diagnostic_updater::Updater diag_updater_;
    std::atomic<bool> closing_;
    ros::Time last_heartbeat_;
    size_t read_errors_;
    size_t write_errors_;
    size_t timeout_errors_;
    size_t open_failures_;
    size_t reconnect_failures_;
    bool link_up_;

    std::mutex mutex_;
};
