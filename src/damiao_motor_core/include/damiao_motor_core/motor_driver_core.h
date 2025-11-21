#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <functional>
#include <vector>
#include <chrono>
#include "transport.h"
#include "motor_state.h"
#include "motor_command.h"
#include "ring_buffer.h"
#include "command_queue.h"
#include "error_handler.h"
#include "config.h"
#include "protocol.h"

namespace damiao_motor_core {

class MotorDriverCore {
public:
    explicit MotorDriverCore(std::shared_ptr<Transport> transport = nullptr);
    ~MotorDriverCore();
    
    // 禁止拷贝和移动
    MotorDriverCore(const MotorDriverCore&) = delete;
    MotorDriverCore& operator=(const MotorDriverCore&) = delete;
    
    // 初始化和配置
    bool initialize(const DriverConfig& config);
    void shutdown();
    
    // 启动/停止
    bool start();
    void stop();
    bool is_running() const { return running_.load(std::memory_order_acquire); }
    
    // 命令发送接口
    enum class CommandStatus {
        Ok,
        Limited,
        SerialError,
        InvalidMotorId
    };
    
    CommandStatus send_command(const MotorCommand& cmd);
    CommandStatus send_commands(const std::vector<MotorCommand>& cmds);
    void send_safe_mode();
    void go_to_zero();
    
    // 状态访问接口（零拷贝）
    template<typename Visitor>
    bool visit_states(Visitor&& visitor);
    
    // 获取最新状态（复制版本，兼容性接口）
    bool get_latest_states(MotorStates& states);
    
    // 统计信息
    uint64_t get_frames_received() const { return frames_received_.load(std::memory_order_acquire); }
    uint64_t get_frames_lost() const { return frames_lost_.load(std::memory_order_acquire); }
    double get_dropout_rate() const;
    double get_avg_feedback_period_ms() const { return feedback_period_avg_ms_.load(std::memory_order_acquire); }
    uint32_t get_last_feedback_age_ms() const;
    
    // 错误处理
    void register_error_callback(std::function<void(const ErrorContext&)> callback);
    
private:
    void feedback_loop();
    void command_loop();
    bool parse_feedback_frame(const uint8_t* buffer, size_t size);
    void update_statistics();
    uint32_t get_timestamp_us() const;
    
    std::shared_ptr<Transport> transport_;
    std::shared_ptr<ErrorHandler> error_handler_;
    DriverConfig config_;
    
    MotorStatesRingBuffer state_buffer_;
    CommandQueue command_queue_;
    
    std::atomic<bool> running_;
    std::thread feedback_thread_;
    std::thread command_thread_;
    
    // 统计信息
    std::atomic<uint64_t> frames_received_;
    std::atomic<uint64_t> frames_lost_;
    std::atomic<uint64_t> checksum_failures_;
    std::atomic<uint64_t> command_limited_count_;
    std::atomic<double> feedback_period_avg_ms_;
    std::atomic<uint32_t> last_feedback_timestamp_us_;
    
    // 帧解析缓冲区
    std::vector<uint8_t> read_buffer_;
    static constexpr size_t kReadChunkSize = 256;
    static constexpr size_t kFailureReconnectThreshold = 5;
    
    // 解析状态
    size_t parse_failure_streak_;
    size_t timeout_streak_;
};

// 模板实现
template<typename Visitor>
bool MotorDriverCore::visit_states(Visitor&& visitor) {
    return state_buffer_.visit_latest(std::forward<Visitor>(visitor));
}

} // namespace damiao_motor_core

