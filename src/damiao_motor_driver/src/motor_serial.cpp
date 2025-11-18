#include "damiao_motor_driver/motor_serial.h"
#include <iostream>

MotorSerial::MotorSerial(std::string port, int baud)
    : port_(port), baud_(baud)
{
}

bool MotorSerial::open()
{
    try {
        ser_.setPort(port_);
        ser_.setBaudrate(baud_);
        serial::Timeout to = serial::Timeout::simpleTimeout(20);
        ser_.setTimeout(to);
        ser_.open();
    } catch (...) {
        return false;
    }
    return ser_.isOpen();
}

bool MotorSerial::is_open()
{
    return ser_.isOpen();
}

size_t MotorSerial::write_bytes(const uint8_t* data, size_t size)
{
    return ser_.write(data, size);
}

size_t MotorSerial::read_bytes(uint8_t* buffer, size_t size)
{
    return ser_.read(buffer, size);
}
