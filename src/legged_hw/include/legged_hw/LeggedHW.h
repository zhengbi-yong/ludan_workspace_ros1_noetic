#pragma once
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>
#include <memory>

#include <dmbot_serial/robot_connect.h>

namespace legged
{

class LeggedHW : public hardware_interface::RobotHW
{
public:
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
  bool loadUrdf(ros::NodeHandle& rootNh);

protected:
  std::shared_ptr<urdf::Model> urdfModel_;
  std::shared_ptr<dmbot_serial::robot> motorsInterface;

  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::PositionJointInterface hybridJointInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
  hardware_interface::JointStateInterface contactSensorInterface_;
};

}  // namespace legged
