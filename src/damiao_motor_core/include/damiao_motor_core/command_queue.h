#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <chrono>
#include "motor_command.h"

namespace damiao_motor_core {

class CommandQueue {
public:
    CommandQueue() = default;
    ~CommandQueue() = default;
    
    // 禁止拷贝和移动
    CommandQueue(const CommandQueue&) = delete;
    CommandQueue& operator=(const CommandQueue&) = delete;
    
    // 添加单个命令
    void push(const MotorCommand& cmd) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(cmd);
        cv_.notify_one();
    }
    
    // 批量添加命令
    void push_batch(const std::vector<MotorCommand>& cmds) {
        if (cmds.empty()) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& cmd : cmds) {
            queue_.push(cmd);
        }
        cv_.notify_one();
    }
    
    // 获取所有待发送命令（非阻塞）
    std::vector<MotorCommand> pop_all() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<MotorCommand> result;
        while (!queue_.empty()) {
            result.push_back(queue_.front());
            queue_.pop();
        }
        return result;
    }
    
    // 获取所有待发送命令（阻塞，带超时）
    std::vector<MotorCommand> pop_all_blocking(uint32_t timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (queue_.empty()) {
            cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms));
        }
        
        std::vector<MotorCommand> result;
        while (!queue_.empty()) {
            result.push_back(queue_.front());
            queue_.pop();
        }
        return result;
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
    }
    
private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<MotorCommand> queue_;
};

} // namespace damiao_motor_core

