# 背景
文件名：2025-11-21_1_damiao-motor-driver-architecture-optimization.md
创建于：2025-11-21_20:48:02
创建者：ludan
主分支：master
任务分支：task/damiao_motor_driver_architecture_optimization_2025-11-21_1
Yolo模式：Ask

# 任务描述
对damiao_motor_driver进行架构优化，包括：
1. 线程安全优化：实现无锁环形缓冲区 + 命令队列机制
2. 性能优化：实现零拷贝状态访问 + 批量命令接口
3. 架构分层：分离核心驱动与ROS包装层
4. 功能分离：将policy_bridge.py独立为独立包
5. 节点统一：实现混合模式（核心驱动节点 + 轻量级桥接）
6. 配置统一：创建统一配置管理类
7. 错误处理：实现统一错误处理框架

# 项目概览
damiao_motor_driver是一个ROS包，用于控制达妙MIT协议电机。当前已能正常工作并支持MoveIt控制，但架构上存在职责边界不清、性能瓶颈、线程安全问题等需要优化。

⚠️ 警告：永远不要修改此部分 ⚠️
核心RIPER-5协议规则：
- 必须在每个响应开头声明模式
- RESEARCH模式：只观察和提问，禁止建议和实施
- INNOVATE模式：只讨论方案，禁止具体规划和实施
- PLAN模式：创建详尽技术规范，禁止实施
- EXECUTE模式：100%忠实执行计划，禁止偏离
- REVIEW模式：逐行验证实施与计划的一致性
⚠️ 警告：永远不要修改此部分 ⚠️

# 分析
通过深入研究代码，发现以下关键问题：
1. MotorDriver职责过重，混合了串口通信、协议解析、ROS发布等功能
2. 线程安全问题：send_cmd()无锁，可能与feedback_loop()冲突
3. 性能问题：get_states()频繁复制，无批量命令接口
4. 架构问题：核心驱动直接依赖ROS，难以测试和替换传输方式
5. 功能混杂：policy_bridge.py包含RL推理、数据记录等高级功能
6. 节点冗余：多个节点重复创建MotorDriver实例
7. 配置分散：配置加载逻辑分散在多个地方

# 提议的解决方案
采用分阶段优化策略：

**阶段1：核心驱动分离（基础架构）**
- 创建damiao_motor_core纯C++库
- 分离传输层、协议层、驱动层
- 移除ROS依赖

**阶段2：性能与线程安全优化**
- 实现无锁环形缓冲区用于状态访问
- 实现命令队列机制
- 添加批量命令接口
- 实现零拷贝状态访问

**阶段3：ROS包装层重构**
- 创建MotorDriverNode作为ROS包装
- 重构MotorHWInterface使用新核心库
- 统一节点架构

**阶段4：功能分离与配置统一**
- 创建独立damiao_policy_bridge包
- 实现统一配置管理类
- 实现统一错误处理框架

# 当前执行步骤："阶段1：核心驱动分离（步骤1-14已完成）"

# 任务进度
[2025-11-21_20:48:02]
- 已修改：创建任务文件
- 更改：初始化任务跟踪
- 原因：开始架构优化任务
- 阻碍因素：无
- 状态：成功

[2025-11-21_21:00:00]
- 已修改：
  - 创建damiao_motor_core包基础结构（CMakeLists.txt, package.xml）
  - 实现传输层抽象（transport.h, transport.cpp）
  - 实现串口传输（serial_transport.h, serial_transport.cpp）
  - 实现协议层（protocol.h, protocol.cpp）
  - 实现数据结构（motor_state.h, motor_command.h）
  - 实现无锁环形缓冲区（ring_buffer.h, ring_buffer.cpp）
  - 实现命令队列（command_queue.h, command_queue.cpp）
  - 实现错误处理框架（error_handler.h, error_handler.cpp）
  - 实现配置管理（config.h, config.cpp）
  - 实现核心驱动类（motor_driver_core.h, motor_driver_core.cpp）
- 更改：完成阶段1的核心驱动分离，创建纯C++库damiao_motor_core
- 原因：分离核心驱动与ROS依赖，为后续优化奠定基础
- 阻碍因素：编译错误（已修复：CMakeLists.txt install顺序、const函数修改atomic变量、IOResult命名空间、未使用变量）
- 状态：成功

[2025-11-21_21:30:00]
- 已修改：
  - 修复damiao_motor_driver中的未使用变量警告（tor_min, tor_max）
  - 创建单元测试文件（test_protocol.cpp, test_ring_buffer.cpp, test_command_queue.cpp, test_motor_driver_core.cpp）
  - 启用CMakeLists.txt中的测试配置
  - 修复测试链接问题（添加pthread库）
- 更改：完成阶段1的单元测试创建和验证
- 原因：确保核心库功能正确性和可用性
- 阻碍因素：测试链接缺少pthread库（已修复）
- 状态：成功

[2025-11-21_21:45:00]
- 已修改：
  - 修复test_ring_buffer.cpp中的有符号/无符号比较警告
  - 修复test_ring_buffer线程安全测试卡死问题（添加超时和睡眠机制）
  - 修复test_motor_driver_core.cpp中的未使用参数警告
- 更改：修复所有编译警告和测试卡死问题
- 原因：确保代码质量和测试稳定性
- 阻碍因素：test_ring_buffer线程安全测试可能无限循环（已修复：添加超时机制和CPU空转避免）
- 状态：成功

[2025-11-21_22:30:00]
- 已修改：
  - 更新damiao_motor_driver的CMakeLists.txt和package.xml，添加对damiao_motor_core的依赖
  - 创建ROS服务定义文件（SendCommand.srv, SendCommands.srv）
  - 创建MotorDriverNode类作为ROS包装层
  - 重构MotorHWInterface使用新的damiao_motor_core::MotorDriverCore
  - 修复编译问题（定时器回调、链接库、-fPIC编译选项）
- 更改：完成阶段2的ROS包装层重构
- 原因：将核心库集成到ROS系统中，实现清晰的架构分层
- 阻碍因素：
  - 定时器回调函数签名问题（已修复：使用lambda表达式）
  - 静态库链接到共享库需要-fPIC（已修复：添加-fPIC编译选项）
  - 库文件安装路径问题（已修复：手动复制库文件）
- 状态：成功

[2025-11-21_23:00:00]
- 已修改：
  - 创建damiao_policy_bridge包（CMakeLists.txt, package.xml）
  - 创建policy_loader.py模块（策略加载：TorchScript/ONNX）
  - 创建data_recorder.py模块（数据记录：rosbag/CSV）
  - 创建action_filter.py模块（动作过滤：平滑和阻尼）
  - 创建policy_bridge_node.py主节点（重构后的完整实现）
  - 创建policy_bridge_example.yaml配置示例
  - 从damiao_motor_driver中移除policy_bridge.py
  - 更新damiao_motor_driver的CMakeLists.txt，移除policy_bridge.py安装
- 更改：完成阶段3的功能分离，将高级应用功能独立为damiao_policy_bridge包
- 原因：分离驱动层和应用层，提高代码模块化和可维护性
- 阻碍因素：无
- 状态：成功

# 详细技术规范

## 阶段1：核心驱动分离（damiao_motor_core）

### 1.1 创建新包结构

**文件路径：** `src/damiao_motor_core/`

**目录结构：**
```
damiao_motor_core/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── damiao_motor_core/
│       ├── transport.h              # 传输抽象接口
│       ├── serial_transport.h        # 串口传输实现
│       ├── protocol.h                # MIT协议编解码
│       ├── motor_state.h             # 电机状态数据结构
│       ├── motor_command.h           # 电机命令数据结构
│       ├── ring_buffer.h             # 无锁环形缓冲区
│       ├── command_queue.h           # 命令队列
│       ├── motor_driver_core.h        # 核心驱动类（无ROS依赖）
│       ├── config.h                  # 配置管理类
│       └── error_handler.h           # 错误处理框架
├── src/
│   ├── transport.cpp
│   ├── serial_transport.cpp
│   ├── protocol.cpp
│   ├── motor_state.cpp
│   ├── motor_command.cpp
│   ├── ring_buffer.cpp
│   ├── command_queue.cpp
│   ├── motor_driver_core.cpp
│   ├── config.cpp
│   └── error_handler.cpp
└── test/
    ├── test_protocol.cpp
    ├── test_ring_buffer.cpp
    ├── test_command_queue.cpp
    └── test_motor_driver_core.cpp
```

### 1.2 传输层抽象（transport.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/transport.h`

**接口定义：**
```cpp
#pragma once

#include <cstdint>
#include <cstddef>

namespace damiao_motor_core {

enum class IOError {
    None = 0,
    NotOpen,
    Timeout,
    IOError,
    InvalidParameter
};

struct IOResult {
    size_t bytes = 0;
    IOError error = IOError::None;
};

class Transport {
public:
    virtual ~Transport() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual IOResult write_bytes(const uint8_t* data, size_t size) = 0;
    virtual IOResult read_bytes(uint8_t* buffer, size_t size) = 0;
};

} // namespace damiao_motor_core
```

### 1.3 协议层（protocol.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/protocol.h`

**接口定义：**
```cpp
#pragma once

#include <cstdint>
#include <cstddef>

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
    uint16_t float_to_uint(float x, float min, float max, int bits);
    float uint_to_float(int x_int, float min, float max, int bits);
    void encode_cmd(uint8_t* tx, uint8_t id, float p, float v, float kp, float kd, float torque);
    bool decode_frame(const uint8_t* rx, size_t size, float* positions, float* velocities, float* torques);
}

} // namespace damiao_motor_core
```

### 1.4 电机状态数据结构（motor_state.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/motor_state.h`

**接口定义：**
```cpp
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
```

### 1.5 电机命令数据结构（motor_command.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/motor_command.h`

**接口定义：**
```cpp
#pragma once

#include <cstdint>

namespace damiao_motor_core {

struct MotorCommand {
    int32_t id = 0;
    float p = 0.0f;
    float v = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;
};

} // namespace damiao_motor_core
```

### 1.6 无锁环形缓冲区（ring_buffer.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/ring_buffer.h`

**接口定义：**
```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include "motor_state.h"

namespace damiao_motor_core {

template<typename T, size_t Size>
class LockFreeRingBuffer {
public:
    LockFreeRingBuffer();
    
    // 写入（生产者，feedback_loop线程）
    bool write(const T& data);
    
    // 读取最新数据（消费者，无锁）
    bool read_latest(T& data) const;
    
    // 访问器模式，零拷贝访问
    template<typename Visitor>
    bool visit_latest(Visitor&& visitor) const;
    
    size_t size() const { return Size; }
    bool empty() const;
    
private:
    alignas(64) std::atomic<size_t> write_index_;  // 缓存行对齐，避免false sharing
    alignas(64) std::atomic<size_t> read_index_;
    T buffer_[Size];
};

// 使用MotorStates作为缓冲区元素类型
using MotorStatesRingBuffer = LockFreeRingBuffer<MotorStates, 2>;

} // namespace damiao_motor_core
```

### 1.7 命令队列（command_queue.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/command_queue.h`

**接口定义：**
```cpp
#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "motor_command.h"

namespace damiao_motor_core {

class CommandQueue {
public:
    CommandQueue();
    ~CommandQueue();
    
    // 添加单个命令
    void push(const MotorCommand& cmd);
    
    // 批量添加命令
    void push_batch(const std::vector<MotorCommand>& cmds);
    
    // 获取所有待发送命令（非阻塞）
    std::vector<MotorCommand> pop_all();
    
    // 获取所有待发送命令（阻塞，带超时）
    std::vector<MotorCommand> pop_all_blocking(uint32_t timeout_ms);
    
    size_t size() const;
    bool empty() const;
    void clear();
    
private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<MotorCommand> queue_;
};

} // namespace damiao_motor_core
```

### 1.8 错误处理框架（error_handler.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/error_handler.h`

**接口定义：**
```cpp
#pragma once

#include <functional>
#include <string>
#include <cstdint>

namespace damiao_motor_core {

enum class ErrorCode {
    None = 0,
    SerialTimeout,
    SerialIOError,
    ChecksumFailure,
    ProtocolError,
    FrameParseError,
    CommandLimited,
    InvalidMotorId,
    TransportNotOpen,
    ReconnectFailed
};

struct ErrorContext {
    ErrorCode code;
    std::string message;
    uint32_t timestamp_us;
    int motor_id;  // -1表示全局错误
};

class ErrorHandler {
public:
    using ErrorCallback = std::function<void(const ErrorContext&)>;
    
    ErrorHandler();
    ~ErrorHandler();
    
    // 报告错误
    void report_error(ErrorCode code, const std::string& message, int motor_id = -1);
    
    // 注册错误回调
    void register_callback(ErrorCode code, ErrorCallback callback);
    void register_global_callback(ErrorCallback callback);
    
    // 获取错误统计
    uint32_t get_error_count(ErrorCode code) const;
    uint32_t get_total_error_count() const;
    void reset_statistics();
    
private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace damiao_motor_core
```

### 1.9 配置管理类（config.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/config.h`

**接口定义：**
```cpp
#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <memory>

namespace damiao_motor_core {

struct MotorConfig {
    int32_t id;
    std::string type;
    int32_t state;
};

struct JointConfig {
    std::string name;
    int32_t motor_id;
    double kp;
    double kd;
};

struct DriverConfig {
    std::string port;
    int32_t baud_rate;
    uint32_t read_timeout_ms;
    uint32_t write_timeout_ms;
    uint32_t reconnect_backoff_ms;
    uint32_t max_reconnect_attempts;
    
    // 电机配置
    std::vector<MotorConfig> motors;
    
    // 关节配置（可选，用于ros_control）
    std::vector<JointConfig> joints;
    
    // 默认增益
    double default_kp;
    double default_kd;
    
    // 限幅
    double kp_limit;
    double kd_limit;
    double torque_limit;
    double velocity_limit;
    double position_limit;
};

class ConfigManager {
public:
    ConfigManager();
    ~ConfigManager();
    
    // 从YAML文件加载
    bool load_from_yaml(const std::string& file_path);
    
    // 从JSON文件加载（可选）
    bool load_from_json(const std::string& file_path);
    
    // 获取配置
    const DriverConfig& get_config() const;
    DriverConfig& get_config();
    
    // 验证配置
    bool validate() const;
    std::string get_validation_errors() const;
    
    // 保存配置
    bool save_to_yaml(const std::string& file_path) const;
    
private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace damiao_motor_core
```

### 1.10 核心驱动类（motor_driver_core.h）

**文件路径：** `src/damiao_motor_core/include/damiao_motor_core/motor_driver_core.h`

**接口定义：**
```cpp
#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <functional>
#include "transport.h"
#include "motor_state.h"
#include "motor_command.h"
#include "ring_buffer.h"
#include "command_queue.h"
#include "error_handler.h"
#include "config.h"

namespace damiao_motor_core {

class MotorDriverCore {
public:
    explicit MotorDriverCore(std::shared_ptr<Transport> transport = nullptr);
    ~MotorDriverCore();
    
    // 初始化和配置
    bool initialize(const DriverConfig& config);
    void shutdown();
    
    // 启动/停止
    bool start();
    void stop();
    bool is_running() const { return running_.load(); }
    
    // 命令发送接口
    enum class CommandStatus {
        Ok,
        Limited,
        SerialError,
        InvalidMotorId
    };
    
    CommandStatus send_command(const MotorCommand& cmd);
    CommandStatus send_commands(const std::vector<MotorCommand>& cmds);
    void send_safe_mode();
    void go_to_zero();
    
    // 状态访问接口（零拷贝）
    template<typename Visitor>
    bool visit_states(Visitor&& visitor) const;
    
    // 获取最新状态（复制版本，兼容性接口）
    bool get_latest_states(MotorStates& states) const;
    
    // 统计信息
    uint64_t get_frames_received() const { return frames_received_; }
    uint64_t get_frames_lost() const { return frames_lost_; }
    double get_dropout_rate() const;
    double get_avg_feedback_period_ms() const { return feedback_period_avg_ms_; }
    uint32_t get_last_feedback_age_ms() const;
    
    // 错误处理
    void register_error_callback(std::function<void(const ErrorContext&)> callback);
    
private:
    void feedback_loop();
    void command_loop();
    bool parse_feedback_frame(const uint8_t* buffer, size_t size);
    void update_statistics();
    
    std::shared_ptr<Transport> transport_;
    std::shared_ptr<ErrorHandler> error_handler_;
    DriverConfig config_;
    
    MotorStatesRingBuffer state_buffer_;
    CommandQueue command_queue_;
    
    std::atomic<bool> running_;
    std::thread feedback_thread_;
    std::thread command_thread_;
    
    // 统计信息
    std::atomic<uint64_t> frames_received_;
    std::atomic<uint64_t> frames_lost_;
    std::atomic<uint64_t> checksum_failures_;
    std::atomic<uint64_t> command_limited_count_;
    std::atomic<double> feedback_period_avg_ms_;
    std::atomic<uint32_t> last_feedback_timestamp_us_;
    
    // 帧解析缓冲区
    std::vector<uint8_t> read_buffer_;
    static constexpr size_t kReadChunkSize = 256;
    static constexpr size_t kFailureReconnectThreshold = 5;
};

// 模板实现
template<typename Visitor>
bool MotorDriverCore::visit_states(Visitor&& visitor) const {
    return state_buffer_.visit_latest(std::forward<Visitor>(visitor));
}

} // namespace damiao_motor_core
```

### 1.11 CMakeLists.txt

**文件路径：** `src/damiao_motor_core/CMakeLists.txt`

**内容规范：**
- 使用C++17标准
- 创建静态库 `damiao_motor_core`
- 不依赖ROS，只依赖标准库和serial库（用于串口）
- 包含单元测试目标

### 1.12 package.xml

**文件路径：** `src/damiao_motor_core/package.xml`

**内容规范：**
- 不依赖ROS包（roscpp等）
- 只依赖serial库（用于串口传输）
- 设置为纯C++库包

## 阶段2：ROS包装层重构

### 2.1 创建MotorDriverNode

**文件路径：** `src/damiao_motor_driver/include/damiao_motor_driver/motor_driver_node.h`

**接口定义：**
```cpp
#pragma once

#include <ros/ros.h>
#include <memory>
#include <damiao_motor_core/motor_driver_core.h>
#include <damiao_motor_control_board_serial/MotorStates.h>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace damiao_motor_driver {

class MotorDriverNode {
public:
    MotorDriverNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~MotorDriverNode();
    
    bool initialize();
    void run();
    void shutdown();
    
private:
    void publish_states();
    void publish_diagnostics();
    void error_callback(const damiao_motor_core::ErrorContext& error);
    
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    std::shared_ptr<damiao_motor_core::MotorDriverCore> driver_core_;
    damiao_motor_core::DriverConfig config_;
    
    ros::Publisher motor_states_pub_;
    ros::Publisher status_pub_;
    ros::Timer state_publish_timer_;
    ros::Timer diagnostic_timer_;
    
    std::string tf_prefix_;
};

} // namespace damiao_motor_driver
```

### 2.2 重构MotorHWInterface

**文件路径：** `src/damiao_motor_driver/include/damiao_motor_driver/motor_hw_interface.h`

**修改内容：**
- 移除MotorDriver直接依赖
- 使用damiao_motor_core::MotorDriverCore
- 通过零拷贝接口访问状态
- 通过批量命令接口发送命令

### 2.3 创建ROS服务接口

**文件路径：** `src/damiao_motor_driver/srv/SendCommand.srv`

**服务定义：**
```
int32 id
float32 p
float32 v
float32 kp
float32 kd
float32 torque
---
bool success
string message
```

**文件路径：** `src/damiao_motor_driver/srv/SendCommands.srv`

**服务定义：**
```
MotorCommand[] commands
---
bool success
string message
```

## 阶段3：功能分离

### 3.1 创建damiao_policy_bridge包

**文件路径：** `src/damiao_policy_bridge/`

**目录结构：**
```
damiao_policy_bridge/
├── CMakeLists.txt
├── package.xml
├── nodes/
│   └── policy_bridge_node.py
├── lib/
│   ├── policy_loader.py
│   ├── data_recorder.py
│   └── action_filter.py
└── config/
    └── policy_bridge_example.yaml
```

**修改内容：**
- 将 `scripts/policy_bridge.py` 移动到新包
- 拆分为多个模块
- 更新导入路径

### 3.2 从damiao_motor_driver移除policy_bridge

**文件路径：** `src/damiao_motor_driver/scripts/policy_bridge.py`

**操作：** 删除此文件

**文件路径：** `src/damiao_motor_driver/CMakeLists.txt`

**修改：** 移除policy_bridge.py的安装规则

## 阶段4：测试节点重构

### 4.1 重构测试节点使用ROS服务

**文件路径：** `src/damiao_motor_driver/src/motor_cmd_bridge.cpp`

**修改内容：**
- 不再直接创建MotorDriver实例
- 通过ROS服务调用motor_driver_node
- 或通过话题订阅motor_states，发布到command话题

### 4.2 移除冗余测试节点

**文件路径：** `src/damiao_motor_driver/src/trajectory_controller_node.cpp`
**文件路径：** `src/damiao_motor_driver/src/send_single_motor_node.cpp`

**操作：** 移动到测试包或工具包，或通过ROS服务/话题接口实现

## 阶段5：配置和启动文件更新

### 5.1 创建统一配置文件格式

**文件路径：** `src/damiao_motor_driver/config/motor_config.yaml`

**格式：**
```yaml
driver:
  port: "/dev/ttyACM0"
  baud_rate: 921600
  read_timeout_ms: 50
  write_timeout_ms: 50
  reconnect_backoff_ms: 500
  max_reconnect_attempts: 5

motors:
  - id: 0
    type: "MIT_4340"
    state: 0
  # ... 更多电机配置

joints:
  - name: "LeftShoulderPitch"
    motor_id: 0
    kp: 20.0
    kd: 1.0
  # ... 更多关节配置

limits:
  kp_limit: 500.0
  kd_limit: 5.0
  torque_limit: 10.0
  velocity_limit: 30.0
  position_limit: 12.5
```

### 5.2 更新启动文件

**文件路径：** `src/damiao_motor_driver/launch/motor_driver.launch`

**修改内容：**
- 使用新的配置文件格式
- 更新参数加载方式

## 实施清单

### 阶段1：核心驱动分离
1. 创建 `src/damiao_motor_core/` 目录结构
2. 创建 `src/damiao_motor_core/CMakeLists.txt`，配置为纯C++库
3. 创建 `src/damiao_motor_core/package.xml`，不依赖ROS
4. 实现 `include/damiao_motor_core/transport.h` 和 `src/transport.cpp`
5. 实现 `include/damiao_motor_core/serial_transport.h` 和 `src/serial_transport.cpp`
6. 实现 `include/damiao_motor_core/protocol.h` 和 `src/protocol.cpp`
7. 实现 `include/damiao_motor_core/motor_state.h`
8. 实现 `include/damiao_motor_core/motor_command.h`
9. 实现 `include/damiao_motor_core/ring_buffer.h` 和 `src/ring_buffer.cpp`
10. 实现 `include/damiao_motor_core/command_queue.h` 和 `src/command_queue.cpp`
11. 实现 `include/damiao_motor_core/error_handler.h` 和 `src/error_handler.cpp`
12. 实现 `include/damiao_motor_core/config.h` 和 `src/config.cpp`
13. 实现 `include/damiao_motor_core/motor_driver_core.h` 和 `src/motor_driver_core.cpp`
14. 创建单元测试文件 `test/test_protocol.cpp`
15. 创建单元测试文件 `test/test_ring_buffer.cpp`
16. 创建单元测试文件 `test/test_command_queue.cpp`
17. 创建单元测试文件 `test/test_motor_driver_core.cpp`
18. 编译并测试damiao_motor_core库

### 阶段2：ROS包装层重构
19. 修改 `src/damiao_motor_driver/CMakeLists.txt`，添加对damiao_motor_core的依赖
20. 修改 `src/damiao_motor_driver/package.xml`，添加对damiao_motor_core的依赖
21. 创建 `include/damiao_motor_driver/motor_driver_node.h`
22. 创建 `src/motor_driver_node.cpp`，实现MotorDriverNode类
23. 修改 `src/motor_driver_node.cpp`（原main函数），使用MotorDriverNode
24. 创建 `srv/SendCommand.srv` 服务定义
25. 创建 `srv/SendCommands.srv` 服务定义
26. 更新 `CMakeLists.txt` 添加服务生成
27. 修改 `include/damiao_motor_driver/motor_hw_interface.h`，使用MotorDriverCore
28. 修改 `src/motor_hw_interface.cpp`，重构为使用MotorDriverCore
29. 更新 `motor_hw_interface_node.cpp`，确保兼容性

### 阶段3：功能分离
30. 创建 `src/damiao_policy_bridge/` 目录结构
31. 创建 `src/damiao_policy_bridge/CMakeLists.txt`
32. 创建 `src/damiao_policy_bridge/package.xml`
33. 创建 `src/damiao_policy_bridge/nodes/policy_bridge_node.py`
34. 创建 `src/damiao_policy_bridge/lib/policy_loader.py`
35. 创建 `src/damiao_motor_driver/lib/data_recorder.py`
36. 创建 `src/damiao_policy_bridge/lib/action_filter.py`
37. 从 `src/damiao_motor_driver/scripts/policy_bridge.py` 复制并重构代码到新包
38. 删除 `src/damiao_motor_driver/scripts/policy_bridge.py`
39. 更新 `src/damiao_motor_driver/CMakeLists.txt`，移除policy_bridge.py安装

### 阶段4：测试节点重构
40. 修改 `src/motor_cmd_bridge.cpp`，使用ROS服务或话题接口
41. 移动 `src/trajectory_controller_node.cpp` 到测试目录或重构为使用服务接口
42. 移动 `src/send_single_motor_node.cpp` 到测试目录或重构为使用服务接口
43. 更新相关启动文件

### 阶段5：配置和启动文件更新
44. 创建 `config/motor_config.yaml` 统一配置文件格式
45. 更新 `launch/motor_driver.launch` 使用新配置格式
46. 更新 `launch/motor_hw_with_moveit.launch` 使用新配置格式
47. 更新所有相关启动文件
48. 更新README.md文档，说明新的架构和使用方式

### 阶段6：测试和验证
49. 编译整个工作空间
50. 运行单元测试
51. 测试motor_driver_node基本功能
52. 测试motor_hw_interface与MoveIt集成
53. 测试policy_bridge独立包
54. 性能测试：验证零拷贝和批量命令的性能提升
55. 线程安全测试：验证无锁缓冲区和命令队列的线程安全性

# 最终审查
待完成
