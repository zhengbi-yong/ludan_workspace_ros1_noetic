#include <gtest/gtest.h>
#include "damiao_motor_core/motor_driver_core.h"
#include "damiao_motor_core/config.h"
#include <memory>
#include <thread>
#include <chrono>

using namespace damiao_motor_core;

// 模拟传输类用于测试
class MockTransport : public Transport {
public:
    bool open() override { return true; }
    void close() override {}
    bool is_open() const override { return true; }
    
    IOResult write_bytes(const uint8_t* /*data*/, size_t size) override {
        IOResult result;
        result.bytes = size;
        result.error = IOError::None;
        return result;
    }
    
    IOResult read_bytes(uint8_t* /*buffer*/, size_t /*size*/) override {
        IOResult result;
        result.bytes = 0;  // 模拟无数据
        result.error = IOError::Timeout;
        return result;
    }
};

TEST(MotorDriverCoreTest, Initialization) {
    auto transport = std::make_shared<MockTransport>();
    MotorDriverCore driver(transport);
    
    DriverConfig config;
    config.port = "/dev/test";
    config.baud_rate = 921600;
    
    EXPECT_TRUE(driver.initialize(config));
    EXPECT_FALSE(driver.is_running());
}

TEST(MotorDriverCoreTest, StartStop) {
    auto transport = std::make_shared<MockTransport>();
    MotorDriverCore driver(transport);
    
    DriverConfig config;
    EXPECT_TRUE(driver.initialize(config));
    
    // 注意：由于MockTransport的read_bytes返回Timeout，start可能会失败
    // 但至少应该能调用而不崩溃
    driver.start();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    driver.stop();
    EXPECT_FALSE(driver.is_running());
}

TEST(MotorDriverCoreTest, SendCommand) {
    auto transport = std::make_shared<MockTransport>();
    MotorDriverCore driver(transport);
    
    DriverConfig config;
    driver.initialize(config);
    driver.start();
    
    MotorCommand cmd;
    cmd.id = 5;
    cmd.p = 1.0f;
    cmd.v = 0.5f;
    cmd.kp = 20.0f;
    cmd.kd = 1.0f;
    cmd.torque = 0.0f;
    
    auto status = driver.send_command(cmd);
    // 由于使用MockTransport，命令应该能加入队列
    EXPECT_NE(status, MotorDriverCore::CommandStatus::SerialError);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    driver.stop();
}

TEST(MotorDriverCoreTest, SendCommandsBatch) {
    auto transport = std::make_shared<MockTransport>();
    MotorDriverCore driver(transport);
    
    DriverConfig config;
    driver.initialize(config);
    driver.start();
    
    std::vector<MotorCommand> cmds;
    for (int i = 0; i < 5; ++i) {
        MotorCommand cmd;
        cmd.id = i;
        cmd.p = static_cast<float>(i);
        cmds.push_back(cmd);
    }
    
    auto status = driver.send_commands(cmds);
    EXPECT_EQ(status, MotorDriverCore::CommandStatus::Ok);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    driver.stop();
}

TEST(MotorDriverCoreTest, InvalidMotorId) {
    auto transport = std::make_shared<MockTransport>();
    MotorDriverCore driver(transport);
    
    DriverConfig config;
    driver.initialize(config);
    
    MotorCommand cmd;
    cmd.id = 100;  // 无效ID
    
    auto status = driver.send_command(cmd);
    EXPECT_EQ(status, MotorDriverCore::CommandStatus::InvalidMotorId);
}

TEST(MotorDriverCoreTest, Statistics) {
    auto transport = std::make_shared<MockTransport>();
    MotorDriverCore driver(transport);
    
    DriverConfig config;
    driver.initialize(config);
    
    EXPECT_EQ(driver.get_frames_received(), 0);
    EXPECT_EQ(driver.get_frames_lost(), 0);
    EXPECT_DOUBLE_EQ(driver.get_dropout_rate(), 0.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

