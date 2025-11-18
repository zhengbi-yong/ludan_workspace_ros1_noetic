#include <gtest/gtest.h>

#include <damiao_motor_driver/motor_hw_interface.h>
#include <damiao_motor_driver/motor_protocol.h>
#include <XmlRpcValue.h>
#include <algorithm>
#include <limits>
#include <queue>
#include <vector>

namespace {
class RecordingTransport : public MotorTransport
{
public:
    RecordingTransport()
        : open_(false), next_read_error_(IOError::Timeout)
    {
    }

    bool open() override
    {
        open_ = true;
        return true;
    }

    void close() override { open_ = false; }

    bool is_open() override { return open_; }

    IOResult write_bytes(const uint8_t* data, size_t size) override
    {
        if (!open_) {
            return {0, IOError::NotOpen};
        }
        writes_.emplace_back(data, data + size);
        return {size, IOError::None};
    }

    IOResult read_bytes(uint8_t* buffer, size_t size) override
    {
        if (!open_) {
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

    const std::vector<std::vector<uint8_t>>& writes() const { return writes_; }

private:
    struct ReadAction
    {
        std::vector<uint8_t> data;
        IOError error;
    };

    bool open_;
    IOError next_read_error_;
    std::queue<ReadAction> read_actions_;
    std::vector<std::vector<uint8_t>> writes_;
};

float DecodeTorque(const std::vector<uint8_t>& tx)
{
    int t_int = ((tx[8] & 0x0F) << 8) | tx[9];
    return MITProtocol::uint_to_float(t_int, MITProtocol::T_MIN, MITProtocol::T_MAX, 12);
}

float DecodeKP(const std::vector<uint8_t>& tx)
{
    int kp_int = (tx[4] << 4) | (tx[5] >> 4);
    return MITProtocol::uint_to_float(kp_int, MITProtocol::KP_MIN, MITProtocol::KP_MAX, 12);
}

float DecodeKD(const std::vector<uint8_t>& tx)
{
    int kd_int = ((tx[5] & 0x0F) << 8) | tx[6];
    return MITProtocol::uint_to_float(kd_int, MITProtocol::KD_MIN, MITProtocol::KD_MAX, 12);
}

XmlRpc::XmlRpcValue BuildSingleJointConfig()
{
    XmlRpc::XmlRpcValue joints;
    joints.setSize(1);
    joints[0]["name"] = "joint_0";
    joints[0]["id"] = 0;
    return joints;
}
}

TEST(MotorHWInterfaceTest, ClampsCommandBeforeSend)
{
    ros::NodeHandle nh("~motor_hw_interface_test_clamp");
    nh.setParam("torque_limit", 1.0);
    nh.setParam("kp_limit", 5.0);
    nh.setParam("kd_limit", 0.2);
    nh.setParam("joints", BuildSingleJointConfig());

    auto transport = std::make_shared<RecordingTransport>();
    MotorHWInterface hw(nh, transport);

    auto* effort_iface = hw.get<hardware_interface::EffortJointInterface>();
    ASSERT_NE(nullptr, effort_iface);
    auto cmd_handle = effort_iface->getHandle("joint_0");
    cmd_handle.setCommand(5.0);  // above torque_limit

    hw.write();
    hw.stopDriver();

    ASSERT_GE(transport->writes().size(), 2u);
    EXPECT_NEAR(1.0, DecodeTorque(transport->writes()[0]), 1e-3);
    EXPECT_FLOAT_EQ(0.0f, DecodeKP(transport->writes().back()));
    EXPECT_FLOAT_EQ(0.0f, DecodeKD(transport->writes().back()));
    EXPECT_FLOAT_EQ(0.0f, DecodeTorque(transport->writes().back()));
}

TEST(MotorHWInterfaceTest, WatchdogTriggersSafeModeWhenStale)
{
    ros::NodeHandle nh("~motor_hw_interface_test_watchdog");
    nh.setParam("joints", BuildSingleJointConfig());

    auto transport = std::make_shared<RecordingTransport>();
    MotorHWInterface hw(nh, transport);

    ros::Duration(0.15).sleep();

    const ros::Duration feedback_timeout(0.05);
    const ros::Duration cmd_timeout(0.05);
    const ros::Time now = ros::Time::now();
    const ros::Time last_feedback = hw.getLastFeedbackTime();
    const ros::Time last_cmd = hw.getLastCommandUpdateTime();
    const ros::Duration feedback_age = last_feedback.isZero()
                                           ? ros::Duration(std::numeric_limits<double>::infinity())
                                           : now - last_feedback;
    const ros::Duration cmd_age = last_cmd.isZero()
                                      ? ros::Duration(std::numeric_limits<double>::infinity())
                                      : now - last_cmd;

    const bool watchdog = (feedback_timeout.toSec() > 0.0 && feedback_age > feedback_timeout) ||
                          (cmd_timeout.toSec() > 0.0 && cmd_age > cmd_timeout);
    if (watchdog) {
        hw.sendSafeMode();
    }
    hw.stopDriver();

    EXPECT_TRUE(watchdog);
    EXPECT_GE(transport->writes().size(), MotorDriver::motor_count());
    EXPECT_FLOAT_EQ(0.0f, DecodeTorque(transport->writes().back()));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_hw_interface_test", ros::init_options::NoSigintHandler);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
