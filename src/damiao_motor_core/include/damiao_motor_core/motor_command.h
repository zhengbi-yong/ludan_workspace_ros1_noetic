#pragma once

#include <cstdint>

namespace damiao_motor_core {

struct MotorCommand {
    int32_t id = 0;
    float p = 0.0f;
    float v = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;
};

} // namespace damiao_motor_core

