#include "damiao_motor_driver/motor_hw_interface.h"

#include <algorithm>
#include <pluginlib/class_list_macros.h>
#include <unordered_map>
#include <XmlRpcValue.h>
namespace damiao_motor_driver {
MotorHWInterface::MotorHWInterface()
    : MotorHWInterface(ros::NodeHandle("~"))
{
}

MotorHWInterface::MotorHWInterface(const ros::NodeHandle& nh, std::shared_ptr<MotorTransport> transport)
    : nh_(nh), driver_(nh, std::move(transport))
{
    limits_.kp = MITProtocol::KP_MAX;
    limits_.kd = MITProtocol::KD_MAX;
    limits_.torque = MITProtocol::T_MAX;
    limits_.velocity = MITProtocol::V_MAX;
    limits_.position = MITProtocol::P_MAX;

    default_kp_ = std::min(20.0, static_cast<double>(MITProtocol::KP_MAX));
    default_kd_ = std::min(1.0, static_cast<double>(MITProtocol::KD_MAX));
    nh_.param("default_kp", default_kp_, default_kp_);
    nh_.param("default_kd", default_kd_, default_kd_);

    nh_.param("kp_limit", limits_.kp, limits_.kp);
    nh_.param("kd_limit", limits_.kd, limits_.kd);
    nh_.param("torque_limit", limits_.torque, limits_.torque);
    nh_.param("velocity_limit", limits_.velocity, limits_.velocity);
    nh_.param("position_limit", limits_.position, limits_.position);

    nh_.param<std::string>("tf_prefix", joint_prefix_, std::string());
    if (!joint_prefix_.empty() && joint_prefix_.back() != '/' && joint_prefix_.back() != '_') {
        joint_prefix_ += "_";
    }

    load_joint_config();

    last_command_update_ = ros::Time::now();
    last_sent_pos_ = cmd_pos_;
    last_sent_vel_ = cmd_vel_;
    last_sent_eff_ = cmd_eff_;

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

    driver_.start();
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
    auto states = driver_.get_states();
    std::unordered_map<int, size_t> id_to_index;
    for (size_t i = 0; i < states.size(); ++i) {
        id_to_index[states[i].id] = i;
    }

    for (size_t i = 0; i < joints_.size(); i++) {
        auto it = id_to_index.find(joints_[i].id);
        if (it == id_to_index.end()) {
            continue;
        }
        const auto& state = states[it->second];
        pos_[i] = state.pos;
        vel_[i] = state.vel;
        eff_[i] = state.tor;
    }
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

    bool cmd_updated = false;

    for (size_t i = 0; i < joints_.size(); i++) {
        bool limited = false;
        const double pos_min = std::max(-limits_copy.position, static_cast<double>(MITProtocol::P_MIN));
        const double pos_max = std::min(limits_copy.position, static_cast<double>(MITProtocol::P_MAX));
        const double vel_min = std::max(-limits_copy.velocity, static_cast<double>(MITProtocol::V_MIN));
        const double vel_max = std::min(limits_copy.velocity, static_cast<double>(MITProtocol::V_MAX));
        const double tor_min = std::max(-limits_copy.torque, static_cast<double>(MITProtocol::T_MIN));
        const double tor_max = std::min(limits_copy.torque, static_cast<double>(MITProtocol::T_MAX));
        const double kp_min = static_cast<double>(MITProtocol::KP_MIN);
        const double kp_max = std::min(limits_copy.kp, static_cast<double>(MITProtocol::KP_MAX));
        const double kd_min = static_cast<double>(MITProtocol::KD_MIN);
        const double kd_max = std::min(limits_copy.kd, static_cast<double>(MITProtocol::KD_MAX));

        double pos_cmd = clamp_value(cmd_pos_[i], pos_min, pos_max, limited);
        double vel_cmd = clamp_value(cmd_vel_[i], vel_min, vel_max, limited);
        double eff_cmd = clamp_value(cmd_eff_[i], tor_min, tor_max, limited);

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

        MotorDriver::CommandStatus status = driver_.send_cmd(joints_[i].id, pos_cmd, vel_cmd, kp_cmd, kd_cmd, eff_cmd);
        if (limited || status != MotorDriver::CommandStatus::Ok) {
            ROS_ERROR_THROTTLE(1.0, "Command to motor %d limited or failed, sending zero torque", joints_[i].id);
            driver_.send_cmd(joints_[i].id, pos_cmd, 0.0, 0.0, 0.0, 0.0);
        }

        if (!cmd_updated) {
            if (i >= last_sent_pos_.size()) {
                cmd_updated = true;
            } else if (pos_cmd != last_sent_pos_[i] || vel_cmd != last_sent_vel_[i] || eff_cmd != last_sent_eff_[i]) {
                cmd_updated = true;
            }
        }

        if (i >= last_sent_pos_.size()) {
            last_sent_pos_.push_back(pos_cmd);
            last_sent_vel_.push_back(vel_cmd);
            last_sent_eff_.push_back(eff_cmd);
        } else {
            last_sent_pos_[i] = pos_cmd;
            last_sent_vel_[i] = vel_cmd;
            last_sent_eff_[i] = eff_cmd;
        }
    }

    if (cmd_updated) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_command_update_ = ros::Time::now();
    }
}

void MotorHWInterface::load_joint_config()
{
    joints_.clear();

    auto append_default = [this]() {
        joints_.resize(MotorDriver::motor_count());
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

    last_sent_pos_ = cmd_pos_;
    last_sent_vel_ = cmd_vel_;
    last_sent_eff_ = cmd_eff_;
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

void MotorHWInterface::sendSafeMode()
{
    driver_.send_safe_mode_frame();
}

void MotorHWInterface::stopDriver()
{
    driver_.stop();
}
}
PLUGINLIB_EXPORT_CLASS(damiao_motor_driver::MotorHWInterface, hardware_interface::RobotHW)