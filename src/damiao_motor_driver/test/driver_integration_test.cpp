#include <gtest/gtest.h>

#include <damiao_motor_driver/motor_driver.h>
#include <damiao_motor_driver/motor_protocol.h>
#include <algorithm>
#include <queue>
#include <vector>

namespace {
class FakeMotorTransport : public MotorTransport
{
public:
    explicit FakeMotorTransport(bool fail_first_open = false)
        : fail_first_open_(fail_first_open), open_attempts_(0), is_open_(false),
          write_error_(IOError::None), next_read_error_(IOError::Timeout)
    {
    }

    bool open() override
    {
        ++open_attempts_;
        if (fail_first_open_ && open_attempts_ == 1) {
            return false;
        }
        is_open_ = true;
        return true;
    }

    void close() override { is_open_ = false; }

    bool is_open() override { return is_open_; }

    IOResult write_bytes(const uint8_t* data, size_t size) override
    {
        if (!is_open_) {
            return {0, IOError::NotOpen};
        }
        if (write_error_ != IOError::None) {
            return {0, write_error_};
        }
        last_write_.assign(data, data + size);
        return {size, IOError::None};
    }

    IOResult read_bytes(uint8_t* buffer, size_t size) override
    {
        if (!is_open_) {
            return {0, IOError::NotOpen};
        }
        if (read_actions_.empty()) {
            return {0, next_read_error_};
        }
        ReadAction action = read_actions_.front();
        read_actions_.pop();
        const size_t to_copy = std::min(size, action.data.size());
        std::copy(action.data.begin(), action.data.begin() + to_copy, buffer);
        return {to_copy, action.error};
    }

    void push_read(const std::vector<uint8_t>& data, IOError error = IOError::None)
    {
        read_actions_.push({data, error});
    }

    void set_next_read_error(IOError error) { next_read_error_ = error; }
    void set_write_error(IOError error) { write_error_ = error; }

    size_t open_attempts() const { return open_attempts_; }
    const std::vector<uint8_t>& last_write() const { return last_write_; }

private:
    struct ReadAction
    {
        std::vector<uint8_t> data;
        IOError error;
    };

    bool fail_first_open_;
    size_t open_attempts_;
    bool is_open_;
    IOError write_error_;
    IOError next_read_error_;
    std::queue<ReadAction> read_actions_;
    std::vector<uint8_t> last_write_;
};

std::vector<uint8_t> BuildFeedbackFrame(float pos, float vel, float torque)
{
    std::vector<uint8_t> frame;
    frame.reserve(1 + 30 * 5 + 1);
    frame.push_back(0x7B);
    const uint16_t p_int = MITProtocol::float_to_uint(pos, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
    const uint16_t v_int = MITProtocol::float_to_uint(vel, MITProtocol::V_MIN, MITProtocol::V_MAX, 12);
    const uint16_t t_int = MITProtocol::float_to_uint(torque, MITProtocol::T_MIN, MITProtocol::T_MAX, 12);
    for (size_t i = 0; i < 30; ++i) {
        frame.push_back(static_cast<uint8_t>(p_int >> 8));
        frame.push_back(static_cast<uint8_t>(p_int & 0xFF));
        frame.push_back(static_cast<uint8_t>(v_int >> 4));
        frame.push_back(static_cast<uint8_t>(((v_int & 0x0F) << 4) | (t_int >> 8)));
        frame.push_back(static_cast<uint8_t>(t_int & 0xFF));
    }
    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size(); ++i) {
        checksum ^= frame[i];
    }
    frame.push_back(checksum);
    return frame;
}

float DecodePosition(const std::vector<uint8_t>& tx)
{
    int p_int = (tx[2] << 8) | tx[3];
    return MITProtocol::uint_to_float(p_int, MITProtocol::P_MIN, MITProtocol::P_MAX, 16);
}

TEST(MotorDriverIntegrationTest, ReconnectAfterTimeout)
{
    ros::NodeHandle nh("~");
    auto fake = std::make_shared<FakeMotorTransport>();
    fake->set_next_read_error(MotorTransport::IOError::Timeout);
    for (int i = 0; i < 5; ++i) {
        fake->push_read({}, MotorTransport::IOError::Timeout);
    }
    fake->push_read(BuildFeedbackFrame(0.0f, 0.0f, 0.0f));

    MotorDriver driver(nh, fake);
    driver.start();
    ros::Duration(0.5).sleep();
    driver.stop();

    EXPECT_GE(fake->open_attempts(), 2u);
}

TEST(MotorDriverIntegrationTest, IgnoreBadChecksumThenUpdate)
{
    ros::NodeHandle nh("~");
    auto fake = std::make_shared<FakeMotorTransport>();
    auto invalid_frame = BuildFeedbackFrame(0.0f, 0.0f, 0.0f);
    invalid_frame.back() ^= 0xFF;
    auto valid_frame = BuildFeedbackFrame(1.0f, 0.5f, -1.0f);
    fake->push_read(invalid_frame);
    fake->push_read(valid_frame);

    MotorDriver driver(nh, fake);
    driver.start();
    ros::Duration(0.5).sleep();
    driver.stop();

    auto states = driver.get_states();
    ASSERT_FALSE(states.empty());
    EXPECT_NEAR(1.0, states[0].pos, 0.01);
    EXPECT_NEAR(0.5, states[0].vel, 0.05);
    EXPECT_NEAR(-1.0, states[0].tor, 0.05);
}

TEST(MotorDriverIntegrationTest, LimitsTriggerAndClampCommand)
{
    ros::NodeHandle nh("~");
    auto fake = std::make_shared<FakeMotorTransport>();
    ASSERT_TRUE(fake->open());

    MotorDriver driver(nh, fake);
    auto status = driver.send_cmd(0, MITProtocol::P_MAX + 10.0f, 0.0f, MITProtocol::KP_MAX + 10.0f,
                                  MITProtocol::KD_MAX + 1.0f, MITProtocol::T_MAX + 2.0f);
    EXPECT_EQ(MotorDriver::CommandStatus::Limited, status);

    const auto& tx = fake->last_write();
    ASSERT_GE(tx.size(), 10u);
    EXPECT_FLOAT_EQ(MITProtocol::P_MAX, DecodePosition(tx));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver_test", ros::init_options::NoSigintHandler);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
