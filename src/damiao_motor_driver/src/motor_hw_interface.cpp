#include "damiao_motor_driver/motor_hw_interface.h"

#include <algorithm>
#include <pluginlib/class_list_macros.h>
#include <unordered_map>
#include <XmlRpcValue.h>
#include <damiao_motor_core/protocol.h>

namespace damiao_motor_driver {
MotorHWInterface::MotorHWInterface()
    : MotorHWInterface(ros::NodeHandle("~"))
{
}

MotorHWInterface::MotorHWInterface(const ros::NodeHandle& nh, std::shared_ptr<damiao_motor_core::Transport> transport)
    : nh_(nh)
{
    zero_srv_ = nh_.advertiseService("go_zero", &MotorHWInterface::goZero, this);

    // 加载配置参数
    std::string port;
    int baud_rate;
    nh_.param<std::string>("port", port, "/dev/mcu");
    nh_.param<int>("baud", baud_rate, 921600);
    
    // 配置核心驱动
    config_.port = port;
    config_.baud_rate = baud_rate;
    config_.read_timeout_ms = 50;
    config_.write_timeout_ms = 50;
    config_.reconnect_backoff_ms = 500;
    config_.max_reconnect_attempts = 5;
    
    // 加载限幅参数
    limits_.kp = damiao_motor_core::MITProtocol::KP_MAX;
    limits_.kd = damiao_motor_core::MITProtocol::KD_MAX;
    limits_.torque = damiao_motor_core::MITProtocol::T_MAX;
    limits_.velocity = damiao_motor_core::MITProtocol::V_MAX;
    limits_.position = damiao_motor_core::MITProtocol::P_MAX;

    default_kp_ = std::min(20.0, static_cast<double>(damiao_motor_core::MITProtocol::KP_MAX));
    default_kd_ = std::min(1.0, static_cast<double>(damiao_motor_core::MITProtocol::KD_MAX));
    nh_.param("default_kp", default_kp_, default_kp_);
    nh_.param("default_kd", default_kd_, default_kd_);

    nh_.param("kp_limit", limits_.kp, limits_.kp);
    nh_.param("kd_limit", limits_.kd, limits_.kd);
    nh_.param("torque_limit", limits_.torque, limits_.torque);
    nh_.param("velocity_limit", limits_.velocity, limits_.velocity);
    nh_.param("position_limit", limits_.position, limits_.position);
    
    config_.kp_limit = limits_.kp;
    config_.kd_limit = limits_.kd;
    config_.torque_limit = limits_.torque;
    config_.velocity_limit = limits_.velocity;
    config_.position_limit = limits_.position;

    nh_.param<std::string>("tf_prefix", joint_prefix_, std::string());
    if (!joint_prefix_.empty() && joint_prefix_.back() != '/' && joint_prefix_.back() != '_') {
        joint_prefix_ += "_";
    }

    load_joint_config();
    
    // 加载电机配置到config_
    config_.motors.resize(30);
    for (int i = 0; i < 30; ++i) {
        config_.motors[i].id = i;
        config_.motors[i].type = "MIT_4340";
        config_.motors[i].state = 0;
    }

    last_command_update_ = ros::Time::now();

    dynamic_reconfigure::Server<damiao_motor_driver::DriverLimitsConfig>::CallbackType cb =
        [this](damiao_motor_driver::DriverLimitsConfig& config, uint32_t) {
            std::lock_guard<std::mutex> lock(limits_mutex_);
            limits_.kp = config.kp_limit;
            limits_.kd = config.kd_limit;
            limits_.torque = config.torque_limit;
            limits_.velocity = config.velocity_limit;
            limits_.position = config.position_limit;
        };
    dr_srv_.setCallback(cb);

    damiao_motor_driver::DriverLimitsConfig cfg;
    cfg.kp_limit = limits_.kp;
    cfg.kd_limit = limits_.kd;
    cfg.torque_limit = limits_.torque;
    cfg.velocity_limit = limits_.velocity;
    cfg.position_limit = limits_.position;
    dr_srv_.updateConfig(cfg);

    // 创建传输层和核心驱动
    if (!transport) {
        transport = std::make_shared<damiao_motor_core::SerialTransport>(port, baud_rate);
    }
    driver_core_ = std::make_shared<damiao_motor_core::MotorDriverCore>(transport);
    
    if (!driver_core_->initialize(config_)) {
        ROS_ERROR("Failed to initialize motor driver core");
        return;
    }
    
    if (!driver_core_->start()) {
        ROS_ERROR("Failed to start motor driver core");
        return;
    }
    for (size_t i = 0; i < joints_.size(); i++)
    {
        hardware_interface::JointStateHandle state_handle(
            joint_prefix_ + joints_[i].name,
            &pos_[i], &vel_[i], &eff_[i]
        );
        jnt_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(state_handle, &cmd_pos_[i]);
        jnt_pos_interface_.registerHandle(pos_handle);

        hardware_interface::JointHandle vel_handle(state_handle, &cmd_vel_[i]);
        jnt_vel_interface_.registerHandle(vel_handle);

        hardware_interface::JointHandle eff_handle(state_handle, &cmd_eff_[i]);
        jnt_eff_interface_.registerHandle(eff_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);
    registerInterface(&jnt_vel_interface_);
    registerInterface(&jnt_eff_interface_);
}

void MotorHWInterface::read()
{
    if (!driver_core_ || !driver_core_->is_running()) {
        return;
    }
    
    // 使用零拷贝访问状态
    driver_core_->visit_states([this](const damiao_motor_core::MotorStates& states) {
        std::unordered_map<int, size_t> id_to_index;
        for (size_t i = 0; i < 30; ++i) {
            id_to_index[states.motors[i].id] = i;
        }

        for (size_t i = 0; i < joints_.size(); i++) {
            auto it = id_to_index.find(joints_[i].id);
            if (it == id_to_index.end()) {
                continue;
            }
            const auto& motor_state = states.motors[it->second];
            pos_[i] = motor_state.pos;
            vel_[i] = motor_state.vel;
            eff_[i] = motor_state.tor;
        }
    });
}

void MotorHWInterface::write()
{
    CommandLimits limits_copy;
    {
        std::lock_guard<std::mutex> lock(limits_mutex_);
        limits_copy = limits_;
    }

    auto clamp_value = [](double value, double min_val, double max_val, bool& limited) {
        double clamped = std::min(std::max(value, min_val), max_val);
        if (clamped != value) {
            limited = true;
        }
        return clamped;
    };

    std::map<std::string, double> kp_params;
    std::map<std::string, double> kd_params;
    refresh_gain_map("joint_kp", kp_params);
    refresh_gain_map("joint_kd", kd_params);

    for (size_t i = 0; i < joints_.size(); i++) {
        bool limited = false;
        const double pos_min = std::max(-limits_copy.position, static_cast<double>(damiao_motor_core::MITProtocol::P_MIN));
        const double pos_max = std::min(limits_copy.position, static_cast<double>(damiao_motor_core::MITProtocol::P_MAX));
        const double vel_min = std::max(-limits_copy.velocity, static_cast<double>(damiao_motor_core::MITProtocol::V_MIN));
        const double vel_max = std::min(limits_copy.velocity, static_cast<double>(damiao_motor_core::MITProtocol::V_MAX));
        const double kp_min = static_cast<double>(damiao_motor_core::MITProtocol::KP_MIN);
        const double kp_max = std::min(limits_copy.kp, static_cast<double>(damiao_motor_core::MITProtocol::KP_MAX));
        const double kd_min = static_cast<double>(damiao_motor_core::MITProtocol::KD_MIN);
        const double kd_max = std::min(limits_copy.kd, static_cast<double>(damiao_motor_core::MITProtocol::KD_MAX));

        // MIT 电机需要同时发送位置、速度、kp、kd 和 torque
        // position_controllers/JointTrajectoryController 通过 PositionJointInterface 发送位置命令
        // 直接使用控制器提供的命令值
        double pos_cmd = clamp_value(cmd_pos_[i], pos_min, pos_max, limited);
        double vel_cmd = clamp_value(cmd_vel_[i], vel_min, vel_max, limited);
        // 对于 position 控制器，effort 命令通常为 0（由电机内部 PID 控制）
        // 如果需要额外的力矩补偿，可以使用 cmd_eff_，否则设为 0
        double eff_cmd = 0.0;  // position 控制模式下，effort 设为 0

        const std::string& joint_name = joints_[i].name;
        double kp_base = joints_[i].kp > 0.0 ? joints_[i].kp : default_kp_;
        double kd_base = joints_[i].kd > 0.0 ? joints_[i].kd : default_kd_;
        if (auto it = kp_params.find(joint_name); it != kp_params.end()) {
            kp_base = it->second;
        }
        if (auto it = kd_params.find(joint_name); it != kd_params.end()) {
            kd_base = it->second;
        }

        double kp_cmd = clamp_value(kp_base, kp_min, kp_max, limited);
        double kd_cmd = clamp_value(kd_base, kd_min, kd_max, limited);

        damiao_motor_core::MotorCommand cmd;
        cmd.id = joints_[i].id;
        cmd.p = pos_cmd;
        cmd.v = vel_cmd;
        cmd.kp = kp_cmd;
        cmd.kd = kd_cmd;
        cmd.torque = eff_cmd;
        
        if (!driver_core_ || !driver_core_->is_running()) {
            continue;
        }
        
        auto status = driver_core_->send_command(cmd);
        if (limited || status != damiao_motor_core::MotorDriverCore::CommandStatus::Ok) {
            ROS_ERROR_THROTTLE(1.0, "Command to motor %d limited or failed, sending zero torque", joints_[i].id);
            cmd.v = 0.0;
            cmd.kp = 0.0;
            cmd.kd = 0.0;
            cmd.torque = 0.0;
            driver_core_->send_command(cmd);
        }

    }

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_command_update_ = ros::Time::now();
}

void MotorHWInterface::load_joint_config()
{
    joints_.clear();

    auto append_default = [this]() {
        joints_.resize(30);  // 默认30个电机
        for (size_t i = 0; i < joints_.size(); ++i) {
            joints_[i].name = "joint_" + std::to_string(i);
            joints_[i].id = static_cast<int>(i);
            joints_[i].kp = default_kp_;
            joints_[i].kd = default_kd_;
        }
    };

    XmlRpc::XmlRpcValue joints_param;
    if (nh_.getParam("joints", joints_param) && joints_param.getType() == XmlRpc::XmlRpcValue::TypeArray && joints_param.size() > 0) {
        joints_.reserve(static_cast<size_t>(joints_param.size()));
        for (int i = 0; i < joints_param.size(); ++i) {
            if (joints_param[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN_STREAM("Ignoring malformed joint entry at index " << i << " in ~joints param");
                continue;
            }
            JointConfig cfg;
            const auto& entry = joints_param[i];
            if (entry.hasMember("name") && entry["name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
                cfg.name = static_cast<std::string>(entry["name"]);
            } else {
                cfg.name = "joint_" + std::to_string(i);
            }

            if (entry.hasMember("id") && (entry["id"].getType() == XmlRpc::XmlRpcValue::TypeInt || entry["id"].getType() == XmlRpc::XmlRpcValue::TypeDouble)) {
                cfg.id = static_cast<int>(entry["id"]);
            } else {
                cfg.id = i;
            }

            cfg.kp = default_kp_;
            cfg.kd = default_kd_;
            if (entry.hasMember("kp") && (entry["kp"].getType() == XmlRpc::XmlRpcValue::TypeInt || entry["kp"].getType() == XmlRpc::XmlRpcValue::TypeDouble)) {
                cfg.kp = static_cast<double>(entry["kp"]);
            }
            if (entry.hasMember("kd") && (entry["kd"].getType() == XmlRpc::XmlRpcValue::TypeInt || entry["kd"].getType() == XmlRpc::XmlRpcValue::TypeDouble)) {
                cfg.kd = static_cast<double>(entry["kd"]);
            }

            joints_.push_back(cfg);
        }
    }

    if (joints_.empty()) {
        append_default();
    }

    cmd_pos_.assign(joints_.size(), 0.0);
    cmd_vel_.assign(joints_.size(), 0.0);
    cmd_eff_.assign(joints_.size(), 0.0);
    pos_.assign(joints_.size(), 0.0);
    vel_.assign(joints_.size(), 0.0);
    eff_.assign(joints_.size(), 0.0);

    last_command_update_ = ros::Time::now();
}

void MotorHWInterface::refresh_gain_map(const std::string& param_name, std::map<std::string, double>& output_map) const
{
    output_map.clear();
    XmlRpc::XmlRpcValue param_value;
    if (!nh_.getParam(param_name, param_value)) {
        return;
    }

    if (param_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_WARN_STREAM("Parameter '" << param_name << "' is not a dictionary; ignoring");
        return;
    }

    for (auto it = param_value.begin(); it != param_value.end(); ++it) {
        if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt || it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            output_map[it->first] = static_cast<double>(it->second);
        }
    }
}

ros::Time MotorHWInterface::getLastCommandUpdateTime() const
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    return last_command_update_;
}

ros::Time MotorHWInterface::getLastFeedbackTime() const
{
    if (!driver_core_) {
        return ros::Time(0);
    }
    // 从核心驱动获取最后反馈时间（需要添加此方法到MotorDriverCore）
    // 暂时返回当前时间
    return ros::Time::now();
}

void MotorHWInterface::sendSafeMode()
{
    if (driver_core_ && driver_core_->is_running()) {
        driver_core_->send_safe_mode();
    }
}

void MotorHWInterface::stopDriver()
{
    if (driver_core_) {
        driver_core_->stop();
    }
}

bool MotorHWInterface::goZero(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    if (!driver_core_ || !driver_core_->is_running()) {
        res.success = false;
        res.message = "Motor driver not running";
        return true;
    }
    
    // 发送零位置命令到所有电机
    std::vector<damiao_motor_core::MotorCommand> cmds;
    for (const auto& joint : joints_) {
        damiao_motor_core::MotorCommand cmd;
        cmd.id = joint.id;
        cmd.p = 0.0;
        cmd.v = 0.0;
        cmd.kp = default_kp_;
        cmd.kd = default_kd_;
        cmd.torque = 0.0;
        cmds.push_back(cmd);
    }
    driver_core_->send_commands(cmds);
    
    res.success = true;
    res.message = "All motors commanded to go to zero.";
    return true;
}

}

PLUGINLIB_EXPORT_CLASS(damiao_motor_driver::MotorHWInterface, hardware_interface::RobotHW)