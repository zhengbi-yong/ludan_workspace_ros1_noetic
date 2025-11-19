#include <controller_manager/controller_manager.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <hardware_interface/robot_hw.h>
#include <limits>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "damiao_motor_driver/motor_hw_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_hw_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    pluginlib::ClassLoader<hardware_interface::RobotHW> loader(
        "damiao_motor_driver", "hardware_interface::RobotHW");

    boost::shared_ptr<hardware_interface::RobotHW> hw;
    try {
        hw = loader.createInstance("damiao_motor_driver/MotorHWInterface");
    }
    catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Failed to create MotorHWInterface: " << ex.what());
        return 1;
    }

    auto motor_hw = boost::dynamic_pointer_cast<MotorHWInterface>(hw);
    if (!motor_hw) {
        ROS_FATAL("Loaded hardware interface does not match MotorHWInterface type");
        return 1;
    }

    controller_manager::ControllerManager cm(motor_hw.get(), nh);

    double feedback_timeout = 0.1; // seconds
    double cmd_timeout = 0.1;      // seconds
    private_nh.param("feedback_timeout", feedback_timeout, feedback_timeout);
    private_nh.param("cmd_timeout", cmd_timeout, cmd_timeout);
    const ros::Duration feedback_timeout_duration(feedback_timeout);
    const ros::Duration cmd_timeout_duration(cmd_timeout);

    diagnostic_updater::Updater diag_updater;
    diag_updater.setHardwareID("motor_hw_interface_control_loop");

    size_t watchdog_trigger_count = 0;
    auto diag_cb = [&](diagnostic_updater::DiagnosticStatusWrapper& stat) {
        const ros::Time now = ros::Time::now();
        const ros::Time last_feedback = motor_hw->getLastFeedbackTime();
        const ros::Time last_cmd = motor_hw->getLastCommandUpdateTime();
        const ros::Duration feedback_age = last_feedback.isZero() ? ros::Duration(std::numeric_limits<double>::infinity()) : now - last_feedback;
        const ros::Duration cmd_age = last_cmd.isZero() ? ros::Duration(std::numeric_limits<double>::infinity()) : now - last_cmd;

        const bool feedback_stale = feedback_timeout_duration.toSec() > 0.0 && feedback_age > feedback_timeout_duration;
        const bool cmd_stale = cmd_timeout_duration.toSec() > 0.0 && cmd_age > cmd_timeout_duration;

        if (feedback_stale || cmd_stale) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Control loop watchdog warning");
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Control loop healthy");
        }

        stat.add("watchdog_triggers", watchdog_trigger_count);
        stat.add("feedback_age", feedback_age.toSec());
        stat.add("cmd_age", cmd_age.toSec());
        stat.add("feedback_timeout", feedback_timeout_duration.toSec());
        stat.add("cmd_timeout", cmd_timeout_duration.toSec());
    };
    diag_updater.add("motor_hw_interface_watchdog", diag_cb);

    bool shutdown_requested = false;
    auto safe_stop_srv = private_nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        "safe_stop",
        [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
            motor_hw->sendSafeMode();
            motor_hw->stopDriver();
            shutdown_requested = true;
            res.success = true;
            res.message = "Safe stop invoked, serial closed.";
            return true;
        });

    double loop_hz = 500.0;
    private_nh.param("loop_hz", loop_hz, loop_hz);
    ros::Rate rate(loop_hz);

    ros::Time last_time = ros::Time::now();
    while (ros::ok() && !shutdown_requested) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const ros::Duration period = now - last_time;
        last_time = now;

        motor_hw->read();
        cm.update(now, period);
        motor_hw->write();

        const ros::Duration feedback_age = motor_hw->getLastFeedbackTime().isZero() ? ros::Duration(std::numeric_limits<double>::infinity())
                                                                                    : now - motor_hw->getLastFeedbackTime();
        const ros::Duration cmd_age = motor_hw->getLastCommandUpdateTime().isZero() ? ros::Duration(std::numeric_limits<double>::infinity())
                                                                                    : now - motor_hw->getLastCommandUpdateTime();

        if ((feedback_timeout_duration.toSec() > 0.0 && feedback_age > feedback_timeout_duration) ||
            (cmd_timeout_duration.toSec() > 0.0 && cmd_age > cmd_timeout_duration)) {
            ++watchdog_trigger_count;
            ROS_WARN_THROTTLE(1.0, "Watchdog triggered: feedback age %.3f s, cmd age %.3f s", feedback_age.toSec(), cmd_age.toSec());
            motor_hw->sendSafeMode();
        }

        diag_updater.update();
        rate.sleep();
    }

    safe_stop_srv.shutdown();
    motor_hw->stopDriver();

    return 0;
}
