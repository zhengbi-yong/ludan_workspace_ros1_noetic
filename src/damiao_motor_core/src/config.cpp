#include "damiao_motor_core/config.h"
#include <fstream>
#include <sstream>

namespace damiao_motor_core {

ConfigManager::ConfigManager() = default;

ConfigManager::~ConfigManager() = default;

const DriverConfig& ConfigManager::get_config() const {
    return config_;
}

DriverConfig& ConfigManager::get_config() {
    return config_;
}

bool ConfigManager::load_from_yaml(const std::string& file_path) {
    // 简化实现：实际应该使用yaml-cpp库
    // 这里提供一个基础框架，后续可以扩展
    std::ifstream file(file_path);
    if (!file.is_open()) {
        validation_errors_ = "Cannot open file: " + file_path;
        return false;
    }
    
    // TODO: 实现YAML解析
    // 目前返回默认配置
    file.close();
    return true;
}

bool ConfigManager::load_from_json(const std::string& file_path) {
    // TODO: 实现JSON解析
    return false;
}

bool ConfigManager::validate() const {
    validation_errors_.clear();
    bool valid = true;
    
    if (config_.port.empty()) {
        validation_errors_ += "Port cannot be empty. ";
        valid = false;
    }
    
    if (config_.baud_rate <= 0) {
        validation_errors_ += "Baud rate must be positive. ";
        valid = false;
    }
    
    if (config_.motors.size() > 30) {
        validation_errors_ += "Too many motors (max 30). ";
        valid = false;
    }
    
    for (const auto& motor : config_.motors) {
        if (motor.id < 0 || motor.id >= 30) {
            validation_errors_ += "Invalid motor ID: " + std::to_string(motor.id) + ". ";
            valid = false;
        }
    }
    
    return valid;
}

std::string ConfigManager::get_validation_errors() const {
    return validation_errors_;
}

bool ConfigManager::save_to_yaml(const std::string& file_path) const {
    // TODO: 实现YAML保存
    return false;
}

} // namespace damiao_motor_core

