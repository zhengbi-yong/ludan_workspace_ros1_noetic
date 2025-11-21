#pragma once

#include <cstdint>
#include <cstddef>

namespace damiao_motor_core {

enum class IOError {
    None = 0,
    NotOpen,
    Timeout,
    IOError,
    InvalidParameter
};

struct IOResult {
    size_t bytes = 0;
    IOError error = IOError::None;
};

class Transport {
public:
    virtual ~Transport() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual IOResult write_bytes(const uint8_t* data, size_t size) = 0;
    virtual IOResult read_bytes(uint8_t* buffer, size_t size) = 0;
};

} // namespace damiao_motor_core

