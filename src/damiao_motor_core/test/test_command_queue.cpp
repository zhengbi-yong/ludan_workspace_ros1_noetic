#include <gtest/gtest.h>
#include "damiao_motor_core/command_queue.h"
#include "damiao_motor_core/motor_command.h"
#include <thread>
#include <chrono>

using namespace damiao_motor_core;

TEST(CommandQueueTest, BasicPushPop) {
    CommandQueue queue;
    
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0);
    
    // 添加命令
    MotorCommand cmd;
    cmd.id = 5;
    cmd.p = 1.0f;
    queue.push(cmd);
    
    EXPECT_FALSE(queue.empty());
    EXPECT_EQ(queue.size(), 1);
    
    // 获取命令
    auto cmds = queue.pop_all();
    EXPECT_EQ(cmds.size(), 1);
    EXPECT_EQ(cmds[0].id, 5);
    EXPECT_FLOAT_EQ(cmds[0].p, 1.0f);
    
    EXPECT_TRUE(queue.empty());
}

TEST(CommandQueueTest, BatchPush) {
    CommandQueue queue;
    
    std::vector<MotorCommand> cmds;
    for (int i = 0; i < 10; ++i) {
        MotorCommand cmd;
        cmd.id = i;
        cmd.p = static_cast<float>(i);
        cmds.push_back(cmd);
    }
    
    queue.push_batch(cmds);
    EXPECT_EQ(queue.size(), 10);
    
    auto popped = queue.pop_all();
    EXPECT_EQ(popped.size(), 10);
    for (size_t i = 0; i < popped.size(); ++i) {
        EXPECT_EQ(popped[i].id, static_cast<int>(i));
    }
}

TEST(CommandQueueTest, Clear) {
    CommandQueue queue;
    
    for (int i = 0; i < 5; ++i) {
        MotorCommand cmd;
        cmd.id = i;
        queue.push(cmd);
    }
    
    EXPECT_EQ(queue.size(), 5);
    queue.clear();
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0);
}

TEST(CommandQueueTest, BlockingPop) {
    CommandQueue queue;
    
    // 在另一个线程中延迟添加命令
    std::thread writer([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        MotorCommand cmd;
        cmd.id = 42;
        queue.push(cmd);
    });
    
    // 阻塞等待命令
    auto cmds = queue.pop_all_blocking(100);
    
    writer.join();
    
    EXPECT_EQ(cmds.size(), 1);
    EXPECT_EQ(cmds[0].id, 42);
}

TEST(CommandQueueTest, ThreadSafety) {
    CommandQueue queue;
    const int num_writers = 4;
    const int commands_per_writer = 100;
    
    std::vector<std::thread> writers;
    for (int i = 0; i < num_writers; ++i) {
        writers.emplace_back([&, i]() {
            for (int j = 0; j < commands_per_writer; ++j) {
                MotorCommand cmd;
                cmd.id = i * 1000 + j;
                queue.push(cmd);
            }
        });
    }
    
    for (auto& t : writers) {
        t.join();
    }
    
    EXPECT_EQ(queue.size(), num_writers * commands_per_writer);
    
    auto all_cmds = queue.pop_all();
    EXPECT_EQ(all_cmds.size(), num_writers * commands_per_writer);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

