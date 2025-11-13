#pragma once
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <thread>
#include <memory>
#include <chrono>

#include "legged_hw/LeggedHW.h"

namespace legged
{

class LeggedHWLoop
{
public:
  LeggedHWLoop(ros::NodeHandle& nh, std::shared_ptr<LeggedHW> hardware_interface);
  ~LeggedHWLoop();

  void update();

private:
  ros::NodeHandle nh_;
  std::shared_ptr<LeggedHW> hardwareInterface_;
  std::shared_ptr<controller_manager::ControllerManager> controllerManager_;

  double loopHz_;
  double cycleTimeErrorThreshold_;

  std::thread loopThread_;
  bool loopRunning_;

  std::chrono::steady_clock::time_point lastTime_;
  ros::Duration elapsedTime_;
  ros::Duration elapsedTime2;
};

}  // namespace legged
