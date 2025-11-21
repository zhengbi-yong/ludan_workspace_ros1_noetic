#include "damiao_motor_core/motor_driver_core.h"
#include "damiao_motor_core/serial_transport.h"
#include <algorithm>
#include <thread>
#include <chrono>

namespace damiao_motor_core {

MotorDriverCore::MotorDriverCore(std::shared_ptr<Transport> transport)
    : transport_(transport ? transport : std::make_shared<SerialTransport>())
    , error_handler_(std::make_shared<ErrorHandler>())
    , running_(false)
    , frames_received_(0)
    , frames_lost_(0)
    , checksum_failures_(0)
    , command_limited_count_(0)
    , feedback_period_avg_ms_(0.0)
    , last_feedback_timestamp_us_(0)
    , parse_failure_streak_(0)
    , timeout_streak_(0)
{
    read_buffer_.reserve(MITProtocol::FRAME_SIZE * 2);
}

MotorDriverCore::~MotorDriverCore() {
    shutdown();
}

uint32_t MotorDriverCore::get_timestamp_us() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

bool MotorDriverCore::initialize(const DriverConfig& config) {
    config_ = config;
    
    // 配置串口传输
    if (auto serial = std::dynamic_pointer_cast<SerialTransport>(transport_)) {
        serial->set_port(config.port);
        serial->set_baud(config.baud_rate);
        serial->set_read_timeout_ms(config.read_timeout_ms);
        serial->set_write_timeout_ms(config.write_timeout_ms);
        serial->set_max_reconnect_attempts(config.max_reconnect_attempts);
        serial->set_reconnect_backoff_ms(config.reconnect_backoff_ms);
    }
    
    return true;
}

void MotorDriverCore::shutdown() {
    stop();
}

bool MotorDriverCore::start() {
    if (running_.load(std::memory_order_acquire)) {
        return true;
    }
    
    if (!transport_->open()) {
        error_handler_->report_error(ErrorCode::TransportNotOpen, "Failed to open transport");
        return false;
    }
    
    running_.store(true, std::memory_order_release);
    feedback_thread_ = std::thread(&MotorDriverCore::feedback_loop, this);
    command_thread_ = std::thread(&MotorDriverCore::command_loop, this);
    
    return true;
}

void MotorDriverCore::stop() {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }
    
    running_.store(false, std::memory_order_release);
    
    if (feedback_thread_.joinable()) {
        feedback_thread_.join();
    }
    
    if (command_thread_.joinable()) {
        command_thread_.join();
    }
    
    send_safe_mode();
    transport_->close();
}

MotorDriverCore::CommandStatus MotorDriverCore::send_command(const MotorCommand& cmd) {
    if (cmd.id < 0 || cmd.id >= 30) {
        error_handler_->report_error(ErrorCode::InvalidMotorId, "Invalid motor ID", cmd.id);
        return CommandStatus::InvalidMotorId;
    }
    
    // 限幅检查
    bool limited = false;
    float p_cmd = std::min(std::max(cmd.p, MITProtocol::P_MIN), MITProtocol::P_MAX);
    float v_cmd = std::min(std::max(cmd.v, MITProtocol::V_MIN), MITProtocol::V_MAX);
    float kp_cmd = std::min(std::max(cmd.kp, MITProtocol::KP_MIN), MITProtocol::KP_MAX);
    float kd_cmd = std::min(std::max(cmd.kd, MITProtocol::KD_MIN), MITProtocol::KD_MAX);
    float torque_cmd = std::min(std::max(cmd.torque, MITProtocol::T_MIN), MITProtocol::T_MAX);
    
    if (p_cmd != cmd.p || v_cmd != cmd.v || kp_cmd != cmd.kp || 
        kd_cmd != cmd.kd || torque_cmd != cmd.torque) {
        limited = true;
        command_limited_count_.fetch_add(1, std::memory_order_relaxed);
    }
    
    // 将命令加入队列（由command_loop线程处理）
    command_queue_.push({cmd.id, p_cmd, v_cmd, kp_cmd, kd_cmd, torque_cmd});
    
    if (limited) {
        return CommandStatus::Limited;
    }
    return CommandStatus::Ok;
}

MotorDriverCore::CommandStatus MotorDriverCore::send_commands(const std::vector<MotorCommand>& cmds) {
    std::vector<MotorCommand> valid_cmds;
    valid_cmds.reserve(cmds.size());
    
    for (const auto& cmd : cmds) {
        if (cmd.id < 0 || cmd.id >= 30) {
            error_handler_->report_error(ErrorCode::InvalidMotorId, "Invalid motor ID", cmd.id);
            continue;
        }
        
        // 限幅
        MotorCommand clamped_cmd = cmd;
        clamped_cmd.p = std::min(std::max(cmd.p, MITProtocol::P_MIN), MITProtocol::P_MAX);
        clamped_cmd.v = std::min(std::max(cmd.v, MITProtocol::V_MIN), MITProtocol::V_MAX);
        clamped_cmd.kp = std::min(std::max(cmd.kp, MITProtocol::KP_MIN), MITProtocol::KP_MAX);
        clamped_cmd.kd = std::min(std::max(cmd.kd, MITProtocol::KD_MIN), MITProtocol::KD_MAX);
        clamped_cmd.torque = std::min(std::max(cmd.torque, MITProtocol::T_MIN), MITProtocol::T_MAX);
        
        valid_cmds.push_back(clamped_cmd);
    }
    
    if (!valid_cmds.empty()) {
        command_queue_.push_batch(valid_cmds);
    }
    
    return CommandStatus::Ok;
}

void MotorDriverCore::send_safe_mode() {
    MotorStates states;
    if (get_latest_states(states)) {
        for (const auto& motor : states.motors) {
            MotorCommand safe_cmd;
            safe_cmd.id = motor.id;
            safe_cmd.p = motor.pos;
            safe_cmd.v = 0.0f;
            safe_cmd.kp = 0.0f;
            safe_cmd.kd = 0.0f;
            safe_cmd.torque = 0.0f;
            command_queue_.push(safe_cmd);
        }
    } else {
        // 如果没有状态，发送零命令给所有电机
        for (int i = 0; i < 30; ++i) {
            MotorCommand safe_cmd;
            safe_cmd.id = i;
            command_queue_.push(safe_cmd);
        }
    }
}

void MotorDriverCore::go_to_zero() {
    for (int i = 0; i < 30; ++i) {
        MotorCommand cmd;
        cmd.id = i;
        cmd.p = 0.0f;
        cmd.v = 0.0f;
        cmd.kp = 20.0f;
        cmd.kd = 1.0f;
        cmd.torque = 0.0f;
        command_queue_.push(cmd);
    }
}

bool MotorDriverCore::get_latest_states(MotorStates& states) {
    return state_buffer_.read_latest(states);
}

double MotorDriverCore::get_dropout_rate() const {
    uint64_t received = frames_received_.load(std::memory_order_acquire);
    uint64_t lost = frames_lost_.load(std::memory_order_acquire);
    uint64_t total = received + lost;
    return (total > 0) ? static_cast<double>(lost) / static_cast<double>(total) : 0.0;
}

uint32_t MotorDriverCore::get_last_feedback_age_ms() const {
    uint32_t last_timestamp = last_feedback_timestamp_us_.load(std::memory_order_acquire);
    if (last_timestamp == 0) {
        return UINT32_MAX;
    }
    uint32_t now = get_timestamp_us();
    return (now - last_timestamp) / 1000;  // 转换为毫秒
}

void MotorDriverCore::register_error_callback(std::function<void(const ErrorContext&)> callback) {
    error_handler_->register_global_callback(callback);
}

void MotorDriverCore::command_loop() {
    while (running_.load(std::memory_order_acquire)) {
        // 从队列获取所有待发送命令
        auto cmds = command_queue_.pop_all_blocking(10);  // 10ms超时
        
        for (const auto& cmd : cmds) {
            uint8_t tx[MITProtocol::CMD_FRAME_SIZE];
            MITProtocol::encode_cmd(tx, static_cast<uint8_t>(cmd.id), 
                                   cmd.p, cmd.v, cmd.kp, cmd.kd, cmd.torque);
            
            auto result = transport_->write_bytes(tx, MITProtocol::CMD_FRAME_SIZE);
            if (result.error != IOError::None) {
                ErrorCode error_code = (result.error == IOError::Timeout) ? 
                    ErrorCode::SerialTimeout : ErrorCode::SerialIOError;
                error_handler_->report_error(error_code, "Failed to send command", cmd.id);
            }
        }
        
        // 控制发送频率（约500Hz）
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void MotorDriverCore::feedback_loop() {
    read_buffer_.clear();
    
    while (running_.load(std::memory_order_acquire)) {
        uint8_t chunk[kReadChunkSize];
        auto result = transport_->read_bytes(chunk, sizeof(chunk));
        
        if (result.error == IOError::Timeout) {
            timeout_streak_++;
            frames_lost_.fetch_add(1, std::memory_order_relaxed);
            if (timeout_streak_ >= kFailureReconnectThreshold) {
                error_handler_->report_error(ErrorCode::SerialTimeout, "Timeout streak exceeded");
                // 尝试重连
                transport_->close();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                transport_->open();
                timeout_streak_ = 0;
            }
            continue;
        }
        
        if (result.error != IOError::None) {
            parse_failure_streak_++;
            error_handler_->report_error(ErrorCode::SerialIOError, "Read error");
            continue;
        }
        
        timeout_streak_ = 0;
        if (result.bytes == 0) {
            continue;
        }
        
        read_buffer_.insert(read_buffer_.end(), chunk, chunk + result.bytes);
        
        // 解析完整帧
        while (read_buffer_.size() >= MITProtocol::FRAME_SIZE) {
            auto header_it = std::find(read_buffer_.begin(), read_buffer_.end(), MITProtocol::FRAME_HEADER);
            if (header_it == read_buffer_.end()) {
                read_buffer_.clear();
                parse_failure_streak_++;
                frames_lost_.fetch_add(read_buffer_.size() / MITProtocol::FRAME_SIZE, std::memory_order_relaxed);
                break;
            }
            
            if (header_it != read_buffer_.begin()) {
                read_buffer_.erase(read_buffer_.begin(), header_it);
                parse_failure_streak_++;
                continue;
            }
            
            if (read_buffer_.size() < MITProtocol::FRAME_SIZE) {
                break;
            }
            
            // 尝试解析帧
            if (parse_feedback_frame(read_buffer_.data(), MITProtocol::FRAME_SIZE)) {
                parse_failure_streak_ = 0;
                frames_received_.fetch_add(1, std::memory_order_relaxed);
            } else {
                parse_failure_streak_++;
                checksum_failures_.fetch_add(1, std::memory_order_relaxed);
                frames_lost_.fetch_add(1, std::memory_order_relaxed);
            }
            
            read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + MITProtocol::FRAME_SIZE);
        }
        
        if (parse_failure_streak_ >= kFailureReconnectThreshold) {
            error_handler_->report_error(ErrorCode::FrameParseError, "Parse failure streak exceeded");
            transport_->close();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            transport_->open();
            parse_failure_streak_ = 0;
        }
    }
}

bool MotorDriverCore::parse_feedback_frame(const uint8_t* buffer, size_t size) {
    float positions[30];
    float velocities[30];
    float torques[30];
    
    if (!MITProtocol::decode_frame(buffer, size, positions, velocities, torques)) {
        return false;
    }
    
    MotorStates states;
    states.timestamp_us = get_timestamp_us();
    states.frame_count = frames_received_.load(std::memory_order_relaxed) + 1;
    
    for (size_t i = 0; i < 30; ++i) {
        states.motors[i].id = (config_.motors.size() > i) ? config_.motors[i].id : static_cast<int32_t>(i);
        states.motors[i].pos = positions[i];
        states.motors[i].vel = velocities[i];
        states.motors[i].tor = torques[i];
        states.motors[i].timestamp_us = states.timestamp_us;
    }
    
    // 写入环形缓冲区
    state_buffer_.write(states);
    
    // 更新统计
    uint32_t now = states.timestamp_us;
    uint32_t last = last_feedback_timestamp_us_.exchange(now, std::memory_order_relaxed);
    if (last > 0) {
        double period_ms = (now - last) / 1000.0;
        double avg = feedback_period_avg_ms_.load(std::memory_order_relaxed);
        if (avg <= 0.0) {
            feedback_period_avg_ms_.store(period_ms, std::memory_order_relaxed);
        } else {
            feedback_period_avg_ms_.store(0.9 * avg + 0.1 * period_ms, std::memory_order_relaxed);
        }
    }
    
    return true;
}

void MotorDriverCore::update_statistics() {
    // 统计信息已通过原子变量实时更新
}

} // namespace damiao_motor_core

