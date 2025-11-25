#pragma once

#include <ros/ros.h>
#include <memory>
#include <damiao_motor_core/motor_driver_core.h>
#include <damiao_motor_core/serial_transport.h>
#include <damiao_motor_control_board_serial/MotorStates.h>
#include <damiao_motor_control_board_serial/MotorState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <damiao_motor_driver/SendCommand.h>
#include <damiao_motor_driver/SendCommands.h>

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
    
    bool send_command_service(
        damiao_motor_driver::SendCommand::Request& req,
        damiao_motor_driver::SendCommand::Response& res);
    
    bool send_commands_service(
        damiao_motor_driver::SendCommands::Request& req,
        damiao_motor_driver::SendCommands::Response& res);
    
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    std::shared_ptr<damiao_motor_core::MotorDriverCore> driver_core_;
    damiao_motor_core::DriverConfig config_;
    
    ros::Publisher motor_states_pub_;
    ros::Publisher status_pub_;
    ros::Timer state_publish_timer_;
    ros::Timer diagnostic_timer_;
    
    ros::ServiceServer send_command_srv_;
    ros::ServiceServer send_commands_srv_;
    
    std::string tf_prefix_;
    
    // 统计信息
    uint64_t last_frames_received_;
    uint64_t last_frames_lost_;
    
    // 优化：预分配的消息对象和缓存数据
    damiao_motor_control_board_serial::MotorStates msg_cache_;
    std::vector<std::string> motor_types_;  // 从配置读取的电机类型
    std::vector<int32_t> motor_states_;     // 从配置读取的电机状态
};

} // namespace damiao_motor_driver

