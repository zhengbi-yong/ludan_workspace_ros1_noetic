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
    
    DriverConfig() 
        : port("/dev/ttyACM0")
        , baud_rate(921600)
        , read_timeout_ms(50)
        , write_timeout_ms(50)
        , reconnect_backoff_ms(500)
        , max_reconnect_attempts(5)
        , default_kp(20.0)
        , default_kd(1.0)
        , kp_limit(500.0)
        , kd_limit(5.0)
        , torque_limit(10.0)
        , velocity_limit(30.0)
        , position_limit(12.5)
    {
        // 初始化默认电机配置（0-29）
        motors.resize(30);
        for (int i = 0; i < 30; ++i) {
            motors[i].id = i;
            motors[i].type = "MIT_4340";
            motors[i].state = 0;
        }
    }
};

class ConfigManager {
public:
    ConfigManager();
    ~ConfigManager();
    
    // 从YAML文件加载（简化版本，实际可以使用yaml-cpp库）
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
    DriverConfig config_;
    mutable std::string validation_errors_;
};

} // namespace damiao_motor_core

