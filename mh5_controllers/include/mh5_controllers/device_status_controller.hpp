
#include <controller_interface/multi_interface_controller.h>


// #include "mh5_controllers/active_joint_controller.hpp"

#pragma once

namespace mh5_controllers
{


class DeviceStatusController : public controller_interface::MultiInterfaceController<hardware_interface::TempVoltInterface, mh5_hardware::VoltCurrInterface>
{
public:
    ExtendedJointTrajectoryController()
    : controller_interface::MultiInterfaceController<hardware_interface::PosVelJointInterface, mh5_hardware::ActiveJointInterface> (true)
    {}

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

private:
    mh5_controllers::BaseJointTrajectoryController*  pos_controller_;
    mh5_controllers::ActiveJointController*          act_controller_;
};