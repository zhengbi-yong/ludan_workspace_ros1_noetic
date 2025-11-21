#pragma once

#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace damiao_motor_core {

namespace MITProtocol {
    // 协议常量
    static constexpr float P_MIN = -12.5f;
    static constexpr float P_MAX = 12.5f;
    static constexpr float V_MIN = -30.0f;
    static constexpr float V_MAX = 30.0f;
    static constexpr float T_MIN = -10.0f;
    static constexpr float T_MAX = 10.0f;
    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;
    
    // 帧格式常量
    static constexpr uint8_t FRAME_HEADER = 0x7B;
    static constexpr size_t MOTOR_DATA_SIZE = 5;
    static constexpr size_t MOTOR_COUNT = 30;
    static constexpr size_t FRAME_SIZE = 1 + MOTOR_COUNT * MOTOR_DATA_SIZE + 1; // 152
    static constexpr size_t CMD_FRAME_SIZE = 11;
    
    // 编码函数
    inline uint16_t float_to_uint(float x, float min, float max, int bits) {
        float clamped = std::min(std::max(x, min), max);
        float span = max - min;
        float scaled = (clamped - min) * ((float)((1 << bits) - 1)) / span;
        return (uint16_t)scaled;
    }
    
    // 解码函数
    inline float uint_to_float(int x_int, float min, float max, int bits) {
        float span = max - min;
        return (float)x_int * span / ((float)((1 << bits) - 1)) + min;
    }
    
    // 编码命令帧
    inline void encode_cmd(
            uint8_t* tx,
            uint8_t id,
            float p, float v,
            float kp, float kd,
            float torque) {
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
    
    // 解码反馈帧
    inline bool decode_frame(const uint8_t* rx, size_t size, float* positions, float* velocities, float* torques) {
        if (size < FRAME_SIZE) {
            return false;
        }
        
        // 检查帧头
        if (rx[0] != FRAME_HEADER) {
            return false;
        }
        
        // 校验和检查
        uint8_t checksum = 0;
        for (size_t i = 0; i < FRAME_SIZE - 1; ++i) {
            checksum ^= rx[i];
        }
        if (checksum != rx[FRAME_SIZE - 1]) {
            return false;
        }
        
        // 解析每个电机的数据
        for (size_t i = 0; i < MOTOR_COUNT; ++i) {
            const uint8_t* p = &rx[1 + i * MOTOR_DATA_SIZE];
            
            int p_int = (p[0] << 8) | p[1];
            int v_int = (p[2] << 4) | (p[3] >> 4);
            int t_int = ((p[3] & 0x0F) << 8) | p[4];
            
            positions[i] = uint_to_float(p_int, P_MIN, P_MAX, 16);
            velocities[i] = uint_to_float(v_int, V_MIN, V_MAX, 12);
            torques[i] = uint_to_float(t_int, T_MIN, T_MAX, 12);
        }
        
        return true;
    }
} // namespace MITProtocol

} // namespace damiao_motor_core

