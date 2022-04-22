#include <realtime_tools/realtime_publisher.h>
// #include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller.h>
#include <mh5_msgs/DeviceStatus.h>
#include <mh5_hardware/resource_interfaces.hpp>


#pragma once

namespace mh5_controllers
{


class DeviceStatusController : public controller_interface::Controller<mh5_hardware::DynamixelStatusInterface>
{
public:
    DeviceStatusController() : rate_(0.0) {}

    bool init(mh5_hardware::DynamixelStatusInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

private:
    double      rate_;
    ros::Time   last_publish_time_;

    std::vector<mh5_hardware::DynamixelStatusHandle>  joints_;
    // std::vector<mh5_hardware::VoltCurrHandle>           sensors_volt_curr_;
    // mh5_controllers::BaseJointTrajectoryController*  pos_controller_;
    // mh5_controllers::ActiveJointController*          act_controller_;
    std::shared_ptr<realtime_tools::RealtimePublisher<mh5_msgs::DeviceStatus> > realtime_pub_;

};

} // namespace