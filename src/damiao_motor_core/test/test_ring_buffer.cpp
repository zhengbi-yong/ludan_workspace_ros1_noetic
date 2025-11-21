#include <gtest/gtest.h>
#include "damiao_motor_core/ring_buffer.h"
#include "damiao_motor_core/motor_state.h"
#include <thread>
#include <vector>
#include <atomic>

using namespace damiao_motor_core;

TEST(RingBufferTest, BasicWriteRead) {
    MotorStatesRingBuffer buffer;
    MotorStates states;
    
    // 初始应该为空
    EXPECT_TRUE(buffer.empty());
    EXPECT_FALSE(buffer.read_latest(states));
    
    // 写入数据
    MotorStates write_states;
    write_states.frame_count = 1;
    write_states.timestamp_us = 1000;
    write_states.motors[0].id = 0;
    write_states.motors[0].pos = 1.5f;
    
    EXPECT_TRUE(buffer.write(write_states));
    EXPECT_FALSE(buffer.empty());
    
    // 读取数据
    MotorStates read_states;
    EXPECT_TRUE(buffer.read_latest(read_states));
    EXPECT_EQ(read_states.frame_count, 1);
    EXPECT_EQ(read_states.timestamp_us, 1000);
    EXPECT_EQ(read_states.motors[0].id, 0);
    EXPECT_FLOAT_EQ(read_states.motors[0].pos, 1.5f);
}

TEST(RingBufferTest, VisitLatest) {
    MotorStatesRingBuffer buffer;
    
    MotorStates write_states;
    write_states.frame_count = 42;
    write_states.motors[5].id = 5;
    write_states.motors[5].pos = 3.14f;
    
    buffer.write(write_states);
    
    // 使用访问器模式
    bool visited = false;
    buffer.visit_latest([&visited](const MotorStates& states) {
        visited = true;
        EXPECT_EQ(states.frame_count, 42);
        EXPECT_FLOAT_EQ(states.motors[5].pos, 3.14f);
    });
    
    EXPECT_TRUE(visited);
}

TEST(RingBufferTest, OverwriteBehavior) {
    MotorStatesRingBuffer buffer;
    
    // 写入多个数据（超过缓冲区大小）
    for (int i = 0; i < 5; ++i) {
        MotorStates states;
        states.frame_count = i;
        buffer.write(states);
    }
    
    // 应该读取到最新的数据
    MotorStates latest;
    EXPECT_TRUE(buffer.read_latest(latest));
    EXPECT_EQ(latest.frame_count, 4);  // 最后一次写入
}

TEST(RingBufferTest, ThreadSafety) {
    MotorStatesRingBuffer buffer;
    std::atomic<int> write_count(0);
    std::atomic<int> read_count(0);
    const int iterations = 1000;
    
    // 写入线程
    std::thread writer([&]() {
        for (int i = 0; i < iterations; ++i) {
            MotorStates states;
            states.frame_count = i;
            buffer.write(states);
            write_count++;
        }
    });
    
    // 读取线程
    std::thread reader([&]() {
        MotorStates states;
        int last_frame = -1;
        int no_data_count = 0;
        const int max_no_data = 10000;  // 防止无限循环
        
        while (read_count < iterations && no_data_count < max_no_data) {
            if (buffer.read_latest(states)) {
                if (static_cast<int>(states.frame_count) > last_frame) {
                    last_frame = static_cast<int>(states.frame_count);
                    read_count++;
                    no_data_count = 0;  // 重置计数器
                }
            } else {
                no_data_count++;
                std::this_thread::sleep_for(std::chrono::microseconds(10));  // 避免CPU空转
            }
        }
    });
    
    writer.join();
    reader.join();
    
    EXPECT_EQ(write_count.load(), iterations);
    EXPECT_GE(read_count.load(), 0);  // 至少读取了一些数据
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

