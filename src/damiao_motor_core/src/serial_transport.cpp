#include "damiao_motor_core/serial_transport.h"
#include <serial/serial.h>
#include <thread>
#include <chrono>

namespace damiao_motor_core {

SerialTransport::SerialTransport(const std::string& port, int baud)
    : port_(port)
    , baud_(baud)
    , read_timeout_ms_(50)
    , write_timeout_ms_(50)
    , reconnect_backoff_ms_(500)
    , reconnect_backoff_max_ms_(5000)
    , max_reconnect_attempts_(5)
    , closing_(false)
{
    serial_ = std::make_unique<serial::Serial>();
}

SerialTransport::~SerialTransport() {
    close();
}

bool SerialTransport::open_once() {
    try {
        serial_->setPort(port_);
        serial_->setBaudrate(static_cast<uint32_t>(baud_));
        
        serial::Timeout timeout(
            read_timeout_ms_, read_timeout_ms_,
            0, write_timeout_ms_, 0
        );
        serial_->setTimeout(timeout);
        
        serial_->open();
        return serial_->isOpen();
    } catch (const serial::IOException& e) {
        return false;
    } catch (const serial::SerialException& e) {
        return false;
    } catch (const std::exception& e) {
        return false;
    }
}

bool SerialTransport::open() {
    closing_.store(false);
    std::unique_lock<std::mutex> lock(mutex_);
    uint32_t backoff = reconnect_backoff_ms_;

    for (int attempt = 1; attempt <= max_reconnect_attempts_; ++attempt) {
        if (open_once()) {
            return true;
        }

        if (attempt == max_reconnect_attempts_) {
            break;
        }

        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff));
        lock.lock();
        backoff = std::min(backoff * 2, reconnect_backoff_max_ms_);
    }

    return false;
}

void SerialTransport::close() {
    closing_.store(true);
    std::lock_guard<std::mutex> lock(mutex_);
    if (serial_ && serial_->isOpen()) {
        serial_->close();
    }
}

bool SerialTransport::is_open() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return serial_ && serial_->isOpen();
}

IOResult SerialTransport::write_bytes(const uint8_t* data, size_t size) {
    IOResult result;
    
    std::unique_lock<std::mutex> lock(mutex_);
    if (!ensure_connection_locked(lock)) {
        result.error = IOError::NotOpen;
        return result;
    }

    try {
        size_t written = serial_->write(data, size);
        result.bytes = written;
        result.error = (written == size) ? IOError::None : IOError::IOError;
    } catch (const serial::IOException& e) {
        result.error = IOError::IOError;
    } catch (const serial::SerialException& e) {
        result.error = IOError::IOError;
    }

    return result;
}

IOResult SerialTransport::read_bytes(uint8_t* buffer, size_t size) {
    IOResult result;
    
    std::unique_lock<std::mutex> lock(mutex_);
    if (!ensure_connection_locked(lock)) {
        result.error = IOError::NotOpen;
        return result;
    }

    try {
        size_t read = serial_->read(buffer, size);
        result.bytes = read;
        if (read == 0) {
            result.error = IOError::Timeout;
        } else {
            result.error = IOError::None;
        }
    } catch (const serial::IOException& e) {
        result.error = IOError::IOError;
    } catch (const serial::SerialException& e) {
        result.error = IOError::IOError;
    }

    return result;
}

bool SerialTransport::ensure_connection_locked(std::unique_lock<std::mutex>& lock) {
    if (closing_.load()) {
        return false;
    }

    if (serial_ && serial_->isOpen()) {
        return true;
    }

    return reconnect_locked(lock);
}

bool SerialTransport::reconnect_locked(std::unique_lock<std::mutex>& lock) {
    uint32_t backoff = reconnect_backoff_ms_;

    for (int attempt = 1; attempt <= max_reconnect_attempts_; ++attempt) {
        if (open_once()) {
            return true;
        }

        if (attempt == max_reconnect_attempts_) {
            break;
        }

        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff));
        lock.lock();
        backoff = std::min(backoff * 2, reconnect_backoff_max_ms_);
    }

    return false;
}

void SerialTransport::update_timeouts_locked() {
    if (serial_) {
        serial::Timeout timeout(
            read_timeout_ms_, read_timeout_ms_,
            0, write_timeout_ms_, 0
        );
        serial_->setTimeout(timeout);
    }
}

} // namespace damiao_motor_core

