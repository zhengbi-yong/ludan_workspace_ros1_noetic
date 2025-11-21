#include "damiao_motor_core/error_handler.h"
#include <chrono>

namespace damiao_motor_core {

ErrorHandler::ErrorHandler() = default;

ErrorHandler::~ErrorHandler() = default;

uint32_t ErrorHandler::get_timestamp_us() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

void ErrorHandler::report_error(ErrorCode code, const std::string& message, int motor_id) {
    ErrorContext context;
    context.code = code;
    context.message = message;
    context.timestamp_us = get_timestamp_us();
    context.motor_id = motor_id;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 更新统计
    error_counts_[code]++;
    
    // 调用特定错误码的回调
    auto it = callbacks_.find(code);
    if (it != callbacks_.end()) {
        for (const auto& callback : it->second) {
            callback(context);
        }
    }
    
    // 调用全局回调
    for (const auto& callback : global_callbacks_) {
        callback(context);
    }
}

void ErrorHandler::register_callback(ErrorCode code, ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_[code].push_back(callback);
}

void ErrorHandler::register_global_callback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    global_callbacks_.push_back(callback);
}

uint32_t ErrorHandler::get_error_count(ErrorCode code) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = error_counts_.find(code);
    return (it != error_counts_.end()) ? it->second : 0;
}

uint32_t ErrorHandler::get_total_error_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    uint32_t total = 0;
    for (const auto& pair : error_counts_) {
        total += pair.second;
    }
    return total;
}

void ErrorHandler::reset_statistics() {
    std::lock_guard<std::mutex> lock(mutex_);
    error_counts_.clear();
}

} // namespace damiao_motor_core

