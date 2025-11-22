#pragma once

#include "transport.h"
#include <string>
#include <mutex>
#include <atomic>
#include <memory>

// 前向声明，避免直接包含serial库头文件
namespace serial {
    class Serial;
}

namespace damiao_motor_core {

class SerialTransport : public Transport {
public:
    SerialTransport(const std::string& port = "/dev/mcu", int baud = 921600);
    ~SerialTransport() override;

    bool open() override;
    void close() override;
    bool is_open() const override;
    IOResult write_bytes(const uint8_t* data, size_t size) override;
    IOResult read_bytes(uint8_t* buffer, size_t size) override;

    // 配置参数
    void set_port(const std::string& port) { port_ = port; }
    void set_baud(int baud) { baud_ = baud; }
    void set_read_timeout_ms(uint32_t timeout_ms) { read_timeout_ms_ = timeout_ms; }
    void set_write_timeout_ms(uint32_t timeout_ms) { write_timeout_ms_ = timeout_ms; }
    void set_max_reconnect_attempts(int attempts) { max_reconnect_attempts_ = attempts; }
    void set_reconnect_backoff_ms(uint32_t backoff_ms) { reconnect_backoff_ms_ = backoff_ms; }

private:
    bool open_once();
    bool ensure_connection_locked(std::unique_lock<std::mutex>& lock);
    bool reconnect_locked(std::unique_lock<std::mutex>& lock);
    void update_timeouts_locked();

    std::unique_ptr<serial::Serial> serial_;
    std::string port_;
    int baud_;
    uint32_t read_timeout_ms_;
    uint32_t write_timeout_ms_;
    uint32_t reconnect_backoff_ms_;
    uint32_t reconnect_backoff_max_ms_;
    int max_reconnect_attempts_;

    std::atomic<bool> closing_;
    mutable std::mutex mutex_;
};

} // namespace damiao_motor_core

