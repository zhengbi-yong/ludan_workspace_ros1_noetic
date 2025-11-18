#include <damiao_motor_driver/motor_driver.h>
#include <algorithm>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iostream>
#include <utility>

namespace {
constexpr size_t kReadChunkSize = 256;
}

MotorDriver::MotorDriver(const ros::NodeHandle& nh, std::shared_ptr<MotorTransport> transport)
    : private_nh_("~"),
      pub_nh_(nh),
      serial_(transport ? std::move(transport) : std::make_shared<MotorSerial>(private_nh_)),
      running_(false),
      checksum_failures_(0),
      dirty_bytes_dropped_(0),
      parse_failure_streak_(0),
      timeout_streak_(0),
      reconnect_requests_(0),
      frames_received_(0),
      frames_lost_(0),
      command_limited_count_(0),
      feedback_period_avg_(0.0),
      last_feedback_time_(ros::Time(0))
{
    ROS_INFO("MotorDriver constructor entered !!!");
    motors_.resize(MOTOR_COUNT);
    load_motor_metadata();

    private_nh_.param<std::string>("tf_prefix", tf_prefix_, std::string());
    if (!tf_prefix_.empty()) {
        pub_nh_ = ros::NodeHandle(nh, tf_prefix_);
    }

    motor_pub_ = pub_nh_.advertise<damiao_motor_control_board_serial::MotorStates>("motor_states", 10);
    status_pub_ = pub_nh_.advertise<diagnostic_msgs::DiagnosticArray>("status", 1, true);

    diag_updater_.setHardwareID("damiao_motor_driver");
    diag_updater_.add("Motor Driver", this, &MotorDriver::populateDiagnostics);
}

void MotorDriver::start()
{
    if (!serial_->open()) {
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

    send_safe_mode_frame();

    serial_->close();
}

namespace
{
float clamp_with_limits(float value, float min_limit, float max_limit, bool& limited)
{
    float clamped = std::min(std::max(value, min_limit), max_limit);
    if (clamped != value)
    {
        limited = true;
    }
    return clamped;
}
} // namespace

MotorDriver::CommandStatus MotorDriver::send_cmd(int id, float p, float v, float kp, float kd, float torque)
{
    bool limited = false;
    float p_cmd = clamp_with_limits(p, MITProtocol::P_MIN, MITProtocol::P_MAX, limited);
    float v_cmd = clamp_with_limits(v, MITProtocol::V_MIN, MITProtocol::V_MAX, limited);
    float kp_cmd = clamp_with_limits(kp, MITProtocol::KP_MIN, MITProtocol::KP_MAX, limited);
    float kd_cmd = clamp_with_limits(kd, MITProtocol::KD_MIN, MITProtocol::KD_MAX, limited);
    float torque_cmd = clamp_with_limits(torque, MITProtocol::T_MIN, MITProtocol::T_MAX, limited);

    uint8_t tx[11];
    MITProtocol::encode_cmd(tx, id, p_cmd, v_cmd, kp_cmd, kd_cmd, torque_cmd);
    MotorTransport::IOResult write_result = serial_->write_bytes(tx, 11);
    if (write_result.error != MotorTransport::IOError::None) {
        ROS_WARN_THROTTLE(2.0, "Failed to send command to motor %d (error %d)", id,
                          static_cast<int>(write_result.error));
        return CommandStatus::SerialError;
    }

    if (limited) {
        ROS_WARN_THROTTLE(2.0, "Command to motor %d was limited to protocol bounds", id);
        ++command_limited_count_;
        publish_status();
        return CommandStatus::Limited;
    }

    return CommandStatus::Ok;
}

std::vector<damiao_motor_control_board_serial::MotorState> MotorDriver::get_states() const
{
    std::lock_guard<std::mutex> lock(motors_mutex_);
    return motors_;
}

void MotorDriver::feedback_loop()
{
    ROS_INFO("feedback_loop started");

    std::vector<uint8_t> buffer;
    buffer.reserve(FRAME_SIZE * 2);

    while (running_ && ros::ok()) {
        uint8_t chunk[kReadChunkSize];
        MotorTransport::IOResult result = serial_->read_bytes(chunk, sizeof(chunk));

        if (result.error == MotorTransport::IOError::Timeout) {
            ++timeout_streak_;
            ++frames_lost_;
            diag_updater_.update();
            if (timeout_streak_ >= FAILURE_RECONNECT_THRESHOLD) {
                ROS_WARN_THROTTLE(2.0, "Motor feedback timeout streak=%zu, requesting reconnect", timeout_streak_);
                ++reconnect_requests_;
                serial_->close();
                if (!serial_->open()) {
                    ROS_ERROR("Failed to reconnect after timeout streak, stopping driver");
                    running_ = false;
                }
            }
            continue;
        }

        if (result.error != MotorTransport::IOError::None) {
            ++parse_failure_streak_;
            ROS_WARN_THROTTLE(2.0, "Serial read error (code %d)", static_cast<int>(result.error));
            diag_updater_.update();
            continue;
        }

        timeout_streak_ = 0;
        if (result.bytes == 0) {
            diag_updater_.update();
            continue;
        }

        buffer.insert(buffer.end(), chunk, chunk + result.bytes);

        bool parsed_frame = false;
        while (buffer.size() >= FRAME_SIZE) {
            auto header_it = std::find(buffer.begin(), buffer.end(), FRAME_HEADER);
            if (header_it == buffer.end()) {
                dirty_bytes_dropped_ += buffer.size();
                frames_lost_ += buffer.size() / FRAME_SIZE;
                buffer.clear();
                ++parse_failure_streak_;
                break;
            }

            if (header_it != buffer.begin()) {
                dirty_bytes_dropped_ += static_cast<size_t>(std::distance(buffer.begin(), header_it));
                buffer.erase(buffer.begin(), header_it);
                ++parse_failure_streak_;
                continue;
            }

            if (buffer.size() < FRAME_SIZE) {
                break;
            }

            uint8_t checksum = 0;
            for (size_t i = 0; i < FRAME_SIZE - 1; ++i) {
                checksum ^= buffer[i];
            }

            if (checksum != buffer[FRAME_SIZE - 1]) {
                ++checksum_failures_;
                ++parse_failure_streak_;
                ++frames_lost_;
                buffer.erase(buffer.begin());
                continue;
            }

            // XOR ok, parse payload
            {
                std::lock_guard<std::mutex> lock(motors_mutex_);
                for (size_t i = 0; i < MOTOR_COUNT; ++i) {
                    const uint8_t* p = &buffer[1 + i * MOTOR_DATA_SIZE];

                    int p_int = (p[0] << 8) | p[1];
                    int v_int = (p[2] << 4) | (p[3] >> 4);
                    int t_int = ((p[3] & 0x0F) << 8) | p[4];

                    motors_[i].pos = MITProtocol::uint_to_float(p_int, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
                    motors_[i].vel = MITProtocol::uint_to_float(v_int, MITProtocol::V_MIN, MITProtocol::V_MAX, 12);
                    motors_[i].tor = MITProtocol::uint_to_float(t_int, MITProtocol::T_MIN, MITProtocol::T_MAX, 12);
                }
                ros::Time now = ros::Time::now();
                if (!last_feedback_time_.isZero()) {
                    double period = (now - last_feedback_time_).toSec();
                    if (feedback_period_avg_ <= 0.0) {
                        feedback_period_avg_ = period;
                    } else {
                        feedback_period_avg_ = 0.9 * feedback_period_avg_ + 0.1 * period;
                    }
                }
                last_feedback_time_ = now;
            }

            buffer.erase(buffer.begin(), buffer.begin() + FRAME_SIZE);
            parse_failure_streak_ = 0;
            parsed_frame = true;
            ++frames_received_;
            publish_states();
        }

        if (!parsed_frame) {
            if (parse_failure_streak_ >= FAILURE_RECONNECT_THRESHOLD) {
                ++reconnect_requests_;
                ROS_WARN_THROTTLE(2.0, "Repeated parse failures (%zu), reconnecting", parse_failure_streak_);
                serial_->close();
                if (!serial_->open()) {
                    ROS_ERROR("Reconnect failed after parse errors, stopping driver");
                    running_ = false;
                }
            }
        }

        diag_updater_.update();
    }
}

void MotorDriver::publish_states()
{
    damiao_motor_control_board_serial::MotorStates msg;
    msg.header.stamp = ros::Time::now();
    msg.motors.resize(MOTOR_COUNT);

    std::vector<damiao_motor_control_board_serial::MotorState> states_copy = get_states();

    for (size_t i = 0; i < states_copy.size(); i++) {
        msg.motors[i].id = states_copy[i].id;
        msg.motors[i].type = states_copy[i].type;
        msg.motors[i].pos = states_copy[i].pos;
        msg.motors[i].vel = states_copy[i].vel;
        msg.motors[i].tor = states_copy[i].tor;
        msg.motors[i].state = states_copy[i].state;
    }

    motor_pub_.publish(msg);
}

void MotorDriver::send_safe_mode_frame()
{
    std::lock_guard<std::mutex> lock(motors_mutex_);
    for (size_t i = 0; i < motors_.size(); ++i) {
        const auto& motor = motors_[i];
        CommandStatus status = send_cmd(motor.id, motor.pos, 0.0f, 0.0f, 0.0f, 0.0f);
        if (status == CommandStatus::SerialError) {
            ROS_ERROR_THROTTLE(2.0, "Failed to send safe-mode frame for motor %d", motor.id);
        }
    }
}

void MotorDriver::load_motor_metadata()
{
    std::lock_guard<std::mutex> lock(motors_mutex_);

    std::vector<int> ids;
    if (private_nh_.getParam("motor_ids", ids) && ids.size() == motors_.size()) {
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i].id = ids[i];
        }
    } else {
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i].id = static_cast<int32_t>(i);
        }
    }

    std::vector<std::string> types;
    if (private_nh_.getParam("motor_types", types) && types.size() == motors_.size()) {
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i].type = types[i];
        }
    } else {
        for (auto& motor : motors_) {
            motor.type = "unknown";
        }
    }

    std::vector<int> states;
    if (private_nh_.getParam("motor_states", states) && states.size() == motors_.size()) {
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i].state = states[i];
        }
    } else {
        for (auto& motor : motors_) {
            motor.state = 0;
        }
    }
}

void MotorDriver::populateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    const bool link_ok = serial_->is_open();
    const bool stale = last_feedback_time_.isZero() || (ros::Time::now() - last_feedback_time_).toSec() > 1.0;

    if (!link_ok || parse_failure_streak_ >= FAILURE_RECONNECT_THRESHOLD || timeout_streak_ >= FAILURE_RECONNECT_THRESHOLD) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor feedback unhealthy");
    } else if (stale) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Motor feedback stale");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor feedback normal");
    }

    stat.add("checksum_failures", checksum_failures_);
    stat.add("dirty_bytes_dropped", dirty_bytes_dropped_);
    stat.add("parse_failure_streak", parse_failure_streak_);
    stat.add("timeout_streak", timeout_streak_);
    stat.add("reconnect_requests", reconnect_requests_);
    stat.add("last_feedback_age", last_feedback_time_.isZero() ? -1.0 : (ros::Time::now() - last_feedback_time_).toSec());
    const double total_frames = static_cast<double>(frames_received_ + frames_lost_);
    stat.add("frames_received", frames_received_);
    stat.add("frames_lost", frames_lost_);
    stat.add("dropout_rate", total_frames > 0.0 ? static_cast<double>(frames_lost_) / total_frames : 0.0);
    stat.add("avg_feedback_period", feedback_period_avg_);
    stat.add("command_limited_count", command_limited_count_);
    publish_status();
}

void MotorDriver::publish_status()
{
    if (!status_pub_) {
        return;
    }

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus status;
    status.name = tf_prefix_.empty() ? "motor_driver" : tf_prefix_ + "/motor_driver";
    status.hardware_id = "damiao_motor_driver";
    status.level = serial_->is_open() ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = serial_->is_open() ? "Serial connected" : "Serial disconnected";
    status.values.reserve(6);
    auto add_key_value = [&status](const std::string& key, const std::string& value) {
        status.values.emplace_back();
        diagnostic_msgs::KeyValue& kv = status.values.back();
        kv.key = key;
        kv.value = value;
    };

    add_key_value("frames_received", std::to_string(frames_received_));
    add_key_value("frames_lost", std::to_string(frames_lost_));
    add_key_value(
        "dropout_rate",
        frames_received_ + frames_lost_ > 0
            ? std::to_string(static_cast<double>(frames_lost_) / static_cast<double>(frames_received_ + frames_lost_))
            : "0");
    add_key_value("reconnect_requests", std::to_string(reconnect_requests_));
    add_key_value("command_limited_count", std::to_string(command_limited_count_));
    add_key_value("avg_feedback_period", std::to_string(feedback_period_avg_));

    array.status.push_back(status);
    status_pub_.publish(array);
}
