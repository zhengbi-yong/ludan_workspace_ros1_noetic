#include "damiao_motor_driver/motor_serial.h"

#include <algorithm>
#include <exception>
#include <utility>

#include <diagnostic_msgs/DiagnosticStatus.h>

MotorSerial::MotorSerial(ros::NodeHandle& nh)
    : port_("/dev/mcu"),
      baud_(921600),
      read_timeout_ms_(50),
      write_timeout_ms_(50),
      heartbeat_timeout_(0.5),
      reconnect_backoff_(0.5),
      reconnect_backoff_max_(5.0),
      max_reconnect_attempts_(5),
      closing_(false),
      last_heartbeat_(ros::Time(0)),
      read_errors_(0),
      write_errors_(0),
      timeout_errors_(0),
      open_failures_(0),
      reconnect_failures_(0),
      link_up_(false)
{
    load_parameters(nh);
    diag_updater_.setHardwareID("damiao_motor_serial");
    diag_updater_.add("Serial Link", this, &MotorSerial::populateDiagnostics);
}

MotorSerial::~MotorSerial()
{
    close();
}

void MotorSerial::load_parameters(ros::NodeHandle& nh)
{
    nh.param<std::string>("port", port_, port_);
    nh.param<int>("baud", baud_, baud_);
    int read_timeout_param = static_cast<int>(read_timeout_ms_);
    int write_timeout_param = static_cast<int>(write_timeout_ms_);
    nh.param<int>("read_timeout_ms", read_timeout_param, read_timeout_param);
    nh.param<int>("write_timeout_ms", write_timeout_param, write_timeout_param);

    read_timeout_ms_ = static_cast<uint32_t>(read_timeout_param);
    write_timeout_ms_ = static_cast<uint32_t>(write_timeout_param);

    double heartbeat_timeout_s = heartbeat_timeout_.toSec();
    double reconnect_backoff_s = reconnect_backoff_.toSec();
    double reconnect_backoff_max_s = reconnect_backoff_max_.toSec();

    nh.param<double>("heartbeat_timeout", heartbeat_timeout_s, heartbeat_timeout_s);
    nh.param<double>("reconnect_backoff", reconnect_backoff_s, reconnect_backoff_s);
    nh.param<double>("reconnect_backoff_max", reconnect_backoff_max_s, reconnect_backoff_max_s);
    nh.param<int>("max_reconnect_attempts", max_reconnect_attempts_, max_reconnect_attempts_);

    heartbeat_timeout_ = ros::Duration(heartbeat_timeout_s);
    reconnect_backoff_ = ros::Duration(reconnect_backoff_s);
    reconnect_backoff_max_ = ros::Duration(reconnect_backoff_max_s);
}

void MotorSerial::update_timeouts_locked()
{
    serial::Timeout to(read_timeout_ms_, read_timeout_ms_, 0, write_timeout_ms_, 0);
    ser_.setTimeout(to);
}

bool MotorSerial::open_once()
{
    try {
        ser_.setPort(port_);
        ser_.setBaudrate(static_cast<uint32_t>(baud_));
        update_timeouts_locked();
        ser_.open();
        last_heartbeat_ = ros::Time::now();
        link_up_ = ser_.isOpen();
    } catch (const serial::IOException& e) {
        ++open_failures_;
        link_up_ = false;
        ROS_WARN_THROTTLE(5.0, "Serial IO exception while opening %s: %s", port_.c_str(), e.what());
    } catch (const serial::SerialException& e) {
        ++open_failures_;
        link_up_ = false;
        ROS_WARN_THROTTLE(5.0, "Serial exception while opening %s: %s", port_.c_str(), e.what());
    } catch (const std::exception& e) {
        ++open_failures_;
        link_up_ = false;
        ROS_WARN_THROTTLE(5.0, "Unknown exception while opening %s: %s", port_.c_str(), e.what());
    }

    diag_updater_.update();
    return ser_.isOpen();
}

bool MotorSerial::open()
{
    closing_.store(false);
    std::unique_lock<std::mutex> lock(mutex_);
    double backoff = reconnect_backoff_.toSec();

    for (int attempt = 1; attempt <= max_reconnect_attempts_; ++attempt) {
        if (open_once()) {
            return true;
        }

        if (attempt == max_reconnect_attempts_) {
            break;
        }

        lock.unlock();
        ros::Duration(backoff).sleep();
        lock.lock();
        backoff = std::min(backoff * 2.0, reconnect_backoff_max_.toSec());
    }

    return false;
}

void MotorSerial::close()
{
    closing_.store(true);
    std::lock_guard<std::mutex> lock(mutex_);
    if (ser_.isOpen()) {
        ser_.close();
    }
    link_up_ = false;
    diag_updater_.update();
}

bool MotorSerial::is_open()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ser_.isOpen();
}

bool MotorSerial::ensure_connection_locked(std::unique_lock<std::mutex>& lock)
{
    if (closing_.load()) {
        return false;
    }

    if (ser_.isOpen()) {
        if (heartbeat_timeout_.isZero()) {
            return true;
        }

        if ((ros::Time::now() - last_heartbeat_) < heartbeat_timeout_) {
            return true;
        }

        link_up_ = false;
        diag_updater_.update();
    }

    return reconnect_locked(lock);
}

bool MotorSerial::reconnect_locked(std::unique_lock<std::mutex>& lock)
{
    double backoff = reconnect_backoff_.toSec();

    for (int attempt = 1; attempt <= max_reconnect_attempts_ && !closing_.load(); ++attempt) {
        if (open_once()) {
            return true;
        }

        ++reconnect_failures_;
        lock.unlock();
        ros::Duration(backoff).sleep();
        lock.lock();
        backoff = std::min(backoff * 2.0, reconnect_backoff_max_.toSec());
    }

    return false;
}

MotorSerial::IOResult MotorSerial::write_bytes(const uint8_t* data, size_t size)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!ensure_connection_locked(lock)) {
        return {0, IOError::NotOpen};
    }

    IOResult result;
    try {
        result.bytes = ser_.write(data, size);
        result.error = IOError::None;
        last_heartbeat_ = ros::Time::now();
        link_up_ = true;
    } catch (const serial::PortNotOpenedException& e) {
        ++write_errors_;
        link_up_ = false;
        ROS_WARN_THROTTLE(2.0, "Port not open during write on %s: %s", port_.c_str(), e.what());
        result.error = IOError::NotOpen;
    } catch (const serial::IOException& e) {
        ++write_errors_;
        link_up_ = false;
        ROS_WARN_THROTTLE(2.0, "IO exception during write on %s: %s", port_.c_str(), e.what());
        result.error = IOError::IOError;
    } catch (const serial::SerialException& e) {
        ++write_errors_;
        link_up_ = false;
        ROS_WARN_THROTTLE(2.0, "Serial exception during write on %s: %s", port_.c_str(), e.what());
        result.error = IOError::IOError;
    }

    diag_updater_.update();
    return result;
}

MotorSerial::IOResult MotorSerial::read_bytes(uint8_t* buffer, size_t size)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!ensure_connection_locked(lock)) {
        return {0, IOError::NotOpen};
    }

    IOResult result;
    try {
        result.bytes = ser_.read(buffer, size);
        if (result.bytes == 0) {
            ++timeout_errors_;
            result.error = IOError::Timeout;
        } else {
            result.error = IOError::None;
            last_heartbeat_ = ros::Time::now();
            link_up_ = true;
        }
    } catch (const serial::PortNotOpenedException& e) {
        ++read_errors_;
        link_up_ = false;
        ROS_WARN_THROTTLE(2.0, "Port not open during read on %s: %s", port_.c_str(), e.what());
        result.error = IOError::NotOpen;
    } catch (const serial::IOException& e) {
        ++read_errors_;
        link_up_ = false;
        ROS_WARN_THROTTLE(2.0, "IO exception during read on %s: %s", port_.c_str(), e.what());
        result.error = IOError::IOError;
    } catch (const serial::SerialException& e) {
        ++read_errors_;
        link_up_ = false;
        ROS_WARN_THROTTLE(2.0, "Serial exception during read on %s: %s", port_.c_str(), e.what());
        result.error = IOError::IOError;
    }

    if (result.error != IOError::None && result.error != IOError::Timeout && !closing_.load()) {
        reconnect_locked(lock);
    }

    diag_updater_.update();
    return result;
}

void MotorSerial::populateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (link_up_) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Serial link active");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Serial link down");
    }

    stat.add("port", port_);
    stat.add("baud", baud_);
    stat.add("open_failures", open_failures_);
    stat.add("reconnect_failures", reconnect_failures_);
    stat.add("read_errors", read_errors_);
    stat.add("write_errors", write_errors_);
    stat.add("timeout_errors", timeout_errors_);
}
