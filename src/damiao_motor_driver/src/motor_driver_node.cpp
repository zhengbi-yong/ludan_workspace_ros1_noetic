#include "damiao_motor_driver/motor_driver_node.h"
#include <damiao_motor_core/config.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

namespace damiao_motor_driver {

MotorDriverNode::MotorDriverNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
    , last_frames_received_(0)
    , last_frames_lost_(0)
{
}

MotorDriverNode::~MotorDriverNode() {
    shutdown();
}

bool MotorDriverNode::initialize() {
    // 加载配置参数
    std::string port;
    int baud_rate;
    private_nh_.param<std::string>("port", port, "/dev/mcu");
    private_nh_.param<int>("baud", baud_rate, 921600);
    private_nh_.param<std::string>("tf_prefix", tf_prefix_, std::string());
    
    // 配置核心驱动
    config_.port = port;
    config_.baud_rate = baud_rate;
    config_.read_timeout_ms = 50;
    config_.write_timeout_ms = 50;
    config_.reconnect_backoff_ms = 500;
    config_.max_reconnect_attempts = 5;
    
    // 加载电机配置
    motor_types_.resize(30, "MIT_4340");  // 默认类型
    motor_states_.resize(30, 0);          // 默认状态
    XmlRpc::XmlRpcValue motors_param;
    if (private_nh_.getParam("motors", motors_param) && motors_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        config_.motors.clear();
        for (int i = 0; i < motors_param.size() && i < 30; ++i) {
            damiao_motor_core::MotorConfig motor_cfg;
            if (motors_param[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                if (motors_param[i].hasMember("id")) {
                    motor_cfg.id = static_cast<int>(motors_param[i]["id"]);
                } else {
                    motor_cfg.id = i;
                }
                if (motors_param[i].hasMember("type")) {
                    motor_cfg.type = static_cast<std::string>(motors_param[i]["type"]);
                    motor_types_[i] = motor_cfg.type;  // 缓存类型
                } else {
                    motor_cfg.type = "MIT_4340";
                    motor_types_[i] = "MIT_4340";
                }
                if (motors_param[i].hasMember("state")) {
                    motor_cfg.state = static_cast<int>(motors_param[i]["state"]);
                    motor_states_[i] = motor_cfg.state;  // 缓存状态
                } else {
                    motor_cfg.state = 0;
                    motor_states_[i] = 0;
                }
            } else {
                motor_cfg.id = i;
                motor_cfg.type = "MIT_4340";
                motor_cfg.state = 0;
                motor_types_[i] = "MIT_4340";
                motor_states_[i] = 0;
            }
            config_.motors.push_back(motor_cfg);
        }
    }
    
    // 预分配消息对象，避免每次分配
    msg_cache_.motors.resize(30);
    
    // 创建传输层
    auto transport = std::make_shared<damiao_motor_core::SerialTransport>(port, baud_rate);
    
    // 创建核心驱动
    driver_core_ = std::make_shared<damiao_motor_core::MotorDriverCore>(transport);
    
    if (!driver_core_->initialize(config_)) {
        ROS_ERROR("Failed to initialize motor driver core");
        return false;
    }
    
    // 注册错误回调
    driver_core_->register_error_callback(
        [this](const damiao_motor_core::ErrorContext& error) {
            this->error_callback(error);
        });
    
    // 设置话题发布（增大队列大小以减少阻塞）
    ros::NodeHandle pub_nh = tf_prefix_.empty() ? nh_ : ros::NodeHandle(nh_, tf_prefix_);
    motor_states_pub_ = pub_nh.advertise<damiao_motor_control_board_serial::MotorStates>("motor_states", 100);
    status_pub_ = pub_nh.advertise<diagnostic_msgs::DiagnosticArray>("status", 1, true);
    
    // 设置服务
    send_command_srv_ = private_nh_.advertiseService("send_command", &MotorDriverNode::send_command_service, this);
    send_commands_srv_ = private_nh_.advertiseService("send_commands", &MotorDriverNode::send_commands_service, this);
    
    // 注册状态更新回调（事件驱动发布，替代Timer）
    bool use_event_driven = true;
    private_nh_.param<bool>("use_event_driven_publish", use_event_driven, true);
    
    if (use_event_driven) {
        // 事件驱动发布：数据到达时立即发布
        driver_core_->register_state_update_callback([this]() {
            this->publish_states();
        });
        ROS_INFO("Using event-driven publishing for motor_states");
    } else {
        // 定时器发布（备用方案）
        double publish_rate;
        private_nh_.param<double>("publish_rate", publish_rate, 500.0);
        state_publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate),
                                              [this](const ros::TimerEvent&) { this->publish_states(); });
        ROS_INFO("Using timer-driven publishing at %.1f Hz", publish_rate);
    }
    
    double diagnostic_rate;
    private_nh_.param<double>("diagnostic_rate", diagnostic_rate, 1.0);
    diagnostic_timer_ = nh_.createTimer(ros::Duration(1.0 / diagnostic_rate),
                                        [this](const ros::TimerEvent&) { this->publish_diagnostics(); });
    
    return true;
}

void MotorDriverNode::run() {
    if (!driver_core_->start()) {
        ROS_ERROR("Failed to start motor driver core");
        return;
    }
    
    ROS_INFO("Motor driver node started");
    ros::spin();
}

void MotorDriverNode::shutdown() {
    if (driver_core_) {
        driver_core_->stop();
    }
}

void MotorDriverNode::publish_states() {
    if (!driver_core_ || !driver_core_->is_running()) {
        return;
    }
    
    // 使用预分配的消息对象，避免每次创建
    msg_cache_.header.stamp = ros::Time::now();
    
    // 使用零拷贝访问状态，直接填充预分配的motors数组
    bool success = driver_core_->visit_states([this](const damiao_motor_core::MotorStates& states) {
        // 直接使用预分配的数组，避免resize开销
        for (size_t i = 0; i < 30; ++i) {
            msg_cache_.motors[i].id = states.motors[i].id;
            msg_cache_.motors[i].pos = states.motors[i].pos;
            msg_cache_.motors[i].vel = states.motors[i].vel;
            msg_cache_.motors[i].tor = states.motors[i].tor;
            // 使用缓存的类型和状态，避免字符串分配
            msg_cache_.motors[i].type = motor_types_[i];
            msg_cache_.motors[i].state = motor_states_[i];
        }
    });
    
    if (success) {
        motor_states_pub_.publish(msg_cache_);
    }
}

void MotorDriverNode::publish_diagnostics() {
    if (!driver_core_) {
        return;
    }
    
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    
    diagnostic_msgs::DiagnosticStatus status;
    status.name = tf_prefix_.empty() ? "motor_driver" : tf_prefix_ + "/motor_driver";
    status.hardware_id = "damiao_motor_driver";
    
    bool is_running = driver_core_->is_running();
    uint64_t frames_received = driver_core_->get_frames_received();
    uint64_t frames_lost = driver_core_->get_frames_lost();
    double dropout_rate = driver_core_->get_dropout_rate();
    uint32_t feedback_age_ms = driver_core_->get_last_feedback_age_ms();
    
    if (!is_running || feedback_age_ms > 1000) {
        status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        status.message = "Motor driver not running or feedback stale";
    } else if (dropout_rate > 0.1 || feedback_age_ms > 100) {
        status.level = diagnostic_msgs::DiagnosticStatus::WARN;
        status.message = "High dropout rate or delayed feedback";
    } else {
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.message = "Motor driver normal";
    }
    
    status.values.push_back(diagnostic_msgs::KeyValue());
    status.values.back().key = "frames_received";
    status.values.back().value = std::to_string(frames_received);
    
    status.values.push_back(diagnostic_msgs::KeyValue());
    status.values.back().key = "frames_lost";
    status.values.back().value = std::to_string(frames_lost);
    
    status.values.push_back(diagnostic_msgs::KeyValue());
    status.values.back().key = "dropout_rate";
    status.values.back().value = std::to_string(dropout_rate);
    
    status.values.push_back(diagnostic_msgs::KeyValue());
    status.values.back().key = "feedback_age_ms";
    status.values.back().value = std::to_string(feedback_age_ms);
    
    array.status.push_back(status);
    status_pub_.publish(array);
}

void MotorDriverNode::error_callback(const damiao_motor_core::ErrorContext& error) {
    ROS_WARN("Motor driver error [%d]: %s (motor_id: %d)", 
             static_cast<int>(error.code), error.message.c_str(), error.motor_id);
}

bool MotorDriverNode::send_command_service(
    damiao_motor_driver::SendCommand::Request& req,
    damiao_motor_driver::SendCommand::Response& res) {
    
    if (!driver_core_ || !driver_core_->is_running()) {
        res.success = false;
        res.message = "Motor driver not running";
        return true;
    }
    
    damiao_motor_core::MotorCommand cmd;
    cmd.id = req.id;
    cmd.p = req.p;
    cmd.v = req.v;
    cmd.kp = req.kp;
    cmd.kd = req.kd;
    cmd.torque = req.torque;
    
    auto status = driver_core_->send_command(cmd);
    
    if (status == damiao_motor_core::MotorDriverCore::CommandStatus::Ok) {
        res.success = true;
        res.message = "Command sent successfully";
    } else if (status == damiao_motor_core::MotorDriverCore::CommandStatus::Limited) {
        res.success = true;
        res.message = "Command sent but was limited";
    } else {
        res.success = false;
        res.message = "Failed to send command";
    }
    
    return true;
}

bool MotorDriverNode::send_commands_service(
    damiao_motor_driver::SendCommands::Request& req,
    damiao_motor_driver::SendCommands::Response& res) {
    
    if (!driver_core_ || !driver_core_->is_running()) {
        res.success = false;
        res.message = "Motor driver not running";
        return true;
    }
    
    std::vector<damiao_motor_core::MotorCommand> cmds;
    cmds.reserve(req.commands.size());
    
    for (const auto& ros_cmd : req.commands) {
        damiao_motor_core::MotorCommand cmd;
        cmd.id = ros_cmd.id;
        cmd.p = ros_cmd.p;
        cmd.v = ros_cmd.v;
        cmd.kp = ros_cmd.kp;
        cmd.kd = ros_cmd.kd;
        cmd.torque = ros_cmd.torque;
        cmds.push_back(cmd);
    }
    
    auto status = driver_core_->send_commands(cmds);
    
    if (status == damiao_motor_core::MotorDriverCore::CommandStatus::Ok) {
        res.success = true;
        res.message = "Commands sent successfully";
    } else {
        res.success = false;
        res.message = "Failed to send commands";
    }
    
    return true;
}

} // namespace damiao_motor_driver
