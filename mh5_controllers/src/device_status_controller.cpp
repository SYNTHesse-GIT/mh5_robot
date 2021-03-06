#include <pluginlib/class_list_macros.hpp>
#include "mh5_controllers/device_status_controller.hpp"

using namespace mh5_controllers;


bool DeviceStatusController::init(mh5_hardware::DynamixelStatusInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
    // get publishing period
    if (!controller_nh.getParam("rate", rate_)){
      ROS_INFO("[%s] Parameter 'rate' not set; default to 1Hz", controller_nh.getNamespace().c_str());
      rate_ = 1.0;
    }

    std::string topic_name;
    if (!controller_nh.getParam("topic", topic_name)){
      ROS_INFO("[%s] Parameter 'topic' not set; default to 'device_status'", controller_nh.getNamespace().c_str());
      topic_name = "device_status";
    }
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<mh5_msgs::DeviceStatus>(root_nh, topic_name, 1));

    std::vector<std::string> joint_names;
    std::string concat_names;
    if (controller_nh.getParam("joints", joint_names)) {
        ROS_INFO("[%s] Joints parameter specified, publishing specified joints in desired order.", controller_nh.getNamespace().c_str());
    } else {
      // get all joint names from the hardware interface
      joint_names = hw->getNames();
    }
    for (auto & joint_name : joint_names) {
        joints_.push_back(hw->getHandle(joint_name));
        realtime_pub_->msg_.device_names.push_back(joint_name);
        realtime_pub_->msg_.temperatures.push_back(0);
        realtime_pub_->msg_.voltages.push_back(0);
        realtime_pub_->msg_.currents.push_back(0);
        realtime_pub_->msg_.active.push_back(0);
        realtime_pub_->msg_.hwerrs.push_back(0);
        concat_names += joint_name + ", ";
    }
    ROS_INFO("[%s] Device Status handles: %d Dynamixel Joints: [%s]", controller_nh.getNamespace().c_str(), joint_names.size(), concat_names.c_str());

    // mh5_hardware::VoltCurrInterface*   volt_curr_hw = robot_hw->get<mh5_hardware::VoltCurrInterface>();
    // if(!volt_curr_hw) {
    //     ROS_ERROR("[DeviceStatusController] Requires volt_curr_interface");
    //     return false;
    // }
    // const std::vector<std::string>& sensor_names = volt_curr_hw->getNames();
    // for (auto & sensor_name : sensor_names)
    // {
    //     sensors_volt_curr_.push_back(volt_curr_hw->getHandle(sensor_name));
    //     realtime_pub_->msg_.device_names.push_back(sensor_name);
    //     realtime_pub_->msg_.temperatures.push_back(0);
    //     realtime_pub_->msg_.voltages.push_back(0);
    //     realtime_pub_->msg_.currents.push_back(0);
    // }

    return true;

}


void DeviceStatusController::starting(const ros::Time& time)
{
    // initialize time
    last_publish_time_ = time;
}


void DeviceStatusController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
    // limit rate of publishing
    if (rate_ > 0.0 && last_publish_time_ + ros::Duration(1/rate_) < time)
    {
        last_publish_time_ = time;
        if (realtime_pub_->trylock()) {
            realtime_pub_->msg_.header.stamp = time;
            for (unsigned i=0; i<joints_.size(); i++) {
                realtime_pub_->msg_.temperatures[i] = joints_[i].getTemperature();
                realtime_pub_->msg_.voltages[i] = joints_[i].getVoltage();
                realtime_pub_->msg_.active[i] = joints_[i].isActive();
                realtime_pub_->msg_.hwerrs[i] = joints_[i].getHWError();
            }
            realtime_pub_->unlockAndPublish();
        }
    } // publish_period
}

void DeviceStatusController::stopping(const ros::Time& /*time*/) {}



PLUGINLIB_EXPORT_CLASS(mh5_controllers::DeviceStatusController, controller_interface::ControllerBase)