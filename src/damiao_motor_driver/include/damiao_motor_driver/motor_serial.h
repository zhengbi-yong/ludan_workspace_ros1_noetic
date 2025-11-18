#pragma once
#include <serial/serial.h>
#include <vector>
#include <string>

class MotorSerial {
public:
    MotorSerial(std::string port, int baud);
    bool open();
    bool is_open();
    size_t write_bytes(const uint8_t* data, size_t size);
    size_t read_bytes(uint8_t* buffer, size_t size);

private:
    serial::Serial ser_;
    std::string port_;
    int baud_;
};
