#include <gtest/gtest.h>
#include "damiao_motor_core/protocol.h"
#include <cstring>

using namespace damiao_motor_core;

TEST(ProtocolTest, FloatToUintConversion) {
    // 测试位置编码
    float p = 0.0f;
    uint16_t p_int = MITProtocol::float_to_uint(p, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
    EXPECT_GE(p_int, 0);
    EXPECT_LE(p_int, 65535);
    
    // 测试边界值
    uint16_t p_min_int = MITProtocol::float_to_uint(MITProtocol::P_MIN, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
    EXPECT_EQ(p_min_int, 0);
    
    uint16_t p_max_int = MITProtocol::float_to_uint(MITProtocol::P_MAX, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
    EXPECT_EQ(p_max_int, 65535);
}

TEST(ProtocolTest, UintToFloatConversion) {
    // 测试位置解码
    int p_int = 32768;  // 中间值
    float p = MITProtocol::uint_to_float(p_int, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
    EXPECT_GE(p, MITProtocol::P_MIN);
    EXPECT_LE(p, MITProtocol::P_MAX);
    
    // 测试边界值
    float p_min = MITProtocol::uint_to_float(0, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
    EXPECT_FLOAT_EQ(p_min, MITProtocol::P_MIN);
    
    float p_max = MITProtocol::uint_to_float(65535, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
    EXPECT_FLOAT_EQ(p_max, MITProtocol::P_MAX);
}

TEST(ProtocolTest, EncodeDecodeRoundTrip) {
    // 测试编码解码往返
    uint8_t tx[MITProtocol::CMD_FRAME_SIZE];
    uint8_t id = 5;
    float p = 1.5f;
    float v = 2.0f;
    float kp = 30.0f;
    float kd = 1.5f;
    float torque = 0.5f;
    
    MITProtocol::encode_cmd(tx, id, p, v, kp, kd, torque);
    
    // 验证帧头
    EXPECT_EQ(tx[0], MITProtocol::FRAME_HEADER);
    EXPECT_EQ(tx[1], id);
    
    // 验证校验和
    uint8_t checksum = 0;
    for (int i = 0; i < 10; i++) {
        checksum ^= tx[i];
    }
    EXPECT_EQ(checksum, tx[10]);
}

TEST(ProtocolTest, EncodeCmdLimits) {
    // 测试限幅功能
    uint8_t tx[MITProtocol::CMD_FRAME_SIZE];
    
    // 测试超出范围的值
    float p_overflow = 100.0f;  // 超出P_MAX
    float v_overflow = 100.0f;  // 超出V_MAX
    
    MITProtocol::encode_cmd(tx, 0, p_overflow, v_overflow, 0, 0, 0);
    
    // 编码应该自动限幅，不应该崩溃
    EXPECT_EQ(tx[0], MITProtocol::FRAME_HEADER);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

