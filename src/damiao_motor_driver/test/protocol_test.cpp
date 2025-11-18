#include <gtest/gtest.h>
#include <damiao_motor_driver/motor_protocol.h>

namespace {
float DecodePosition(const uint8_t* tx)
{
    int p_int = (tx[2] << 8) | tx[3];
    return MITProtocol::uint_to_float(p_int, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
}

float DecodeVelocity(const uint8_t* tx)
{
    int v_int = (tx[4] << 4) | (tx[5] >> 4);
    return MITProtocol::uint_to_float(v_int, MITProtocol::V_MIN, MITProtocol::V_MAX, 12);
}

float DecodeTorque(const uint8_t* tx)
{
    int t_int = ((tx[8] & 0x0F) << 8) | tx[9];
    return MITProtocol::uint_to_float(t_int, MITProtocol::T_MIN, MITProtocol::T_MAX, 12);
}
}  // namespace

TEST(MotorProtocolTest, ClampAndChecksum)
{
    uint8_t tx[11] = {};
    MITProtocol::encode_cmd(tx, 1, MITProtocol::P_MAX + 5.0f, MITProtocol::V_MIN - 10.0f,
                            MITProtocol::KP_MAX + 50.0f, MITProtocol::KD_MIN - 1.0f,
                            MITProtocol::T_MAX + 5.0f);

    uint8_t checksum = 0;
    for (int i = 0; i < 10; ++i) {
        checksum ^= tx[i];
    }

    EXPECT_EQ(0x7B, tx[0]);
    EXPECT_EQ(checksum, tx[10]);
    EXPECT_FLOAT_EQ(MITProtocol::P_MAX, DecodePosition(tx));
    EXPECT_FLOAT_EQ(MITProtocol::V_MIN, DecodeVelocity(tx));
    EXPECT_FLOAT_EQ(MITProtocol::T_MAX, DecodeTorque(tx));
}

TEST(MotorProtocolTest, RoundTripEncoding)
{
    uint8_t tx[11] = {};
    const float pos = 1.5f;
    const float vel = -5.0f;
    const float torque = 3.0f;

    MITProtocol::encode_cmd(tx, 5, pos, vel, 100.0f, 2.5f, torque);

    EXPECT_NEAR(pos, DecodePosition(tx), 0.001f);
    EXPECT_NEAR(vel, DecodeVelocity(tx), 0.01f);
    EXPECT_NEAR(torque, DecodeTorque(tx), 0.01f);
}
