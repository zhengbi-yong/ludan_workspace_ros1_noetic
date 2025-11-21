#pragma once

#include <functional>
#include <string>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <mutex>

namespace damiao_motor_core {

enum class ErrorCode {
    None = 0,
    SerialTimeout,
    SerialIOError,
    ChecksumFailure,
    ProtocolError,
    FrameParseError,
    CommandLimited,
    InvalidMotorId,
    TransportNotOpen,
    ReconnectFailed
};

struct ErrorContext {
    ErrorCode code;
    std::string message;
    uint32_t timestamp_us;
    int motor_id;  // -1表示全局错误
};

class ErrorHandler {
public:
    using ErrorCallback = std::function<void(const ErrorContext&)>;
    
    ErrorHandler();
    ~ErrorHandler();
    
    // 报告错误
    void report_error(ErrorCode code, const std::string& message, int motor_id = -1);
    
    // 注册错误回调
    void register_callback(ErrorCode code, ErrorCallback callback);
    void register_global_callback(ErrorCallback callback);
    
    // 获取错误统计
    uint32_t get_error_count(ErrorCode code) const;
    uint32_t get_total_error_count() const;
    void reset_statistics();
    
private:
    std::unordered_map<ErrorCode, std::vector<ErrorCallback>> callbacks_;
    std::vector<ErrorCallback> global_callbacks_;
    std::unordered_map<ErrorCode, uint32_t> error_counts_;
    mutable std::mutex mutex_;
    
    uint32_t get_timestamp_us() const;
};

} // namespace damiao_motor_core

