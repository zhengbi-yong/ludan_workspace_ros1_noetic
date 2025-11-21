#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include "motor_state.h"

namespace damiao_motor_core {

template<typename T, size_t Size>
class LockFreeRingBuffer {
public:
    LockFreeRingBuffer() : write_index_(0), read_index_(0) {
        // 初始化缓冲区
        for (size_t i = 0; i < Size; ++i) {
            // 使用placement new初始化（如果需要）
        }
    }
    
    // 写入（生产者，feedback_loop线程）
    bool write(const T& data) {
        size_t current_write = write_index_.load(std::memory_order_relaxed);
        size_t next_write = (current_write + 1) % Size;
        
        // 检查是否有空间（简单版本：如果read_index追上write_index，说明满了）
        // 注意：这个实现允许覆盖，确保总是有最新数据
        buffer_[current_write] = data;
        write_index_.store(next_write, std::memory_order_release);
        return true;
    }
    
    // 读取最新数据（消费者，无锁）
    bool read_latest(T& data) {
        size_t current_read = read_index_.load(std::memory_order_acquire);
        size_t current_write = write_index_.load(std::memory_order_acquire);
        
        if (current_read == current_write) {
            // 缓冲区为空或未初始化
            return false;
        }
        
        // 读取最新的数据（write_index - 1位置）
        size_t latest_index = (current_write + Size - 1) % Size;
        data = buffer_[latest_index];
        
        // 更新read_index到最新位置
        read_index_.store(current_write, std::memory_order_release);
        return true;
    }
    
    // 访问器模式，零拷贝访问
    template<typename Visitor>
    bool visit_latest(Visitor&& visitor) {
        size_t current_read = read_index_.load(std::memory_order_acquire);
        size_t current_write = write_index_.load(std::memory_order_acquire);
        
        if (current_read == current_write) {
            return false;
        }
        
        size_t latest_index = (current_write + Size - 1) % Size;
        visitor(buffer_[latest_index]);
        read_index_.store(current_write, std::memory_order_release);
        return true;
    }
    
    size_t size() const { return Size; }
    
    bool empty() const {
        size_t current_read = read_index_.load(std::memory_order_acquire);
        size_t current_write = write_index_.load(std::memory_order_acquire);
        return current_read == current_write;
    }
    
private:
    alignas(64) std::atomic<size_t> write_index_;  // 缓存行对齐，避免false sharing
    alignas(64) std::atomic<size_t> read_index_;
    T buffer_[Size];
};

// 使用MotorStates作为缓冲区元素类型
using MotorStatesRingBuffer = LockFreeRingBuffer<MotorStates, 2>;

} // namespace damiao_motor_core

