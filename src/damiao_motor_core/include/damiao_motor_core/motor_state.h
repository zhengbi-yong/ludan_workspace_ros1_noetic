#pragma once

#include <cstdint>

namespace damiao_motor_core {

struct MotorState {
    int32_t id = 0;
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    uint32_t timestamp_us = 0;  // 微秒时间戳
};

struct MotorStates {
    MotorState motors[30];  // 固定大小数组，避免动态分配
    uint32_t frame_count = 0;
    uint32_t timestamp_us = 0;
};

} // namespace damiao_motor_core

