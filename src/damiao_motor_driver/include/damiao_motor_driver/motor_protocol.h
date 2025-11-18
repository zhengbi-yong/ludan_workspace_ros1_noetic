#pragma once

#include <cstdint>
#include <algorithm>

namespace MITProtocol {

// ======== 4340 电机编码范围 ========
static constexpr float P_MIN = -12.5f;
static constexpr float P_MAX =  12.5f;

static constexpr float V_MIN = -30.0f;
static constexpr float V_MAX =  30.0f;

static constexpr float T_MIN = -10.0f;
static constexpr float T_MAX =  10.0f;

static constexpr float KP_MIN = 0.0f;
static constexpr float KP_MAX = 500.0f;

static constexpr float KD_MIN = 0.0f;
static constexpr float KD_MAX = 5.0f;

// =======================================================
// float → uint
// =======================================================
inline uint16_t float_to_uint(float x, float min, float max, int bits)
{
    float clamped = std::min(std::max(x, min), max);
    float span = max - min;
    float scaled = (clamped - min) * ((float)((1 << bits) - 1)) / span;
    return (uint16_t)scaled;
}

// =======================================================
// uint → float
// =======================================================
inline float uint_to_float(int x_int, float min, float max, int bits)
{
    float span = max - min;
    return (float)x_int * span / ((float)((1 << bits) - 1)) + min;
}

// =======================================================
// 编码 11 字节 MIT 帧
// =======================================================
inline void encode_cmd(
        uint8_t* tx,
        uint8_t id,
        float p, float v,
        float kp, float kd,
        float torque)
{
    uint16_t p_int  = float_to_uint(p,  P_MIN,  P_MAX,  16);
    uint16_t v_int  = float_to_uint(v,  V_MIN,  V_MAX,  12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t_int  = float_to_uint(torque, T_MIN, T_MAX, 12);

    tx[0] = 0x7B;
    tx[1] = id;

    tx[2] = p_int >> 8;
    tx[3] = p_int & 0xFF;

    tx[4] = v_int >> 4;
    tx[5] = ((v_int & 0x0F) << 4) | (kp_int >> 8);

    tx[6] = kp_int & 0xFF;

    tx[7] = kd_int >> 4;
    tx[8] = ((kd_int & 0x0F) << 4) | (t_int >> 8);

    tx[9] = t_int & 0xFF;

    // XOR 校验
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum ^= tx[i];
    tx[10] = sum;
}

} // namespace MITProtocol
