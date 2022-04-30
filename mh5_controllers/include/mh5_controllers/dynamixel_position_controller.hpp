#include <std_srvs/Trigger.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/multi_interface_controller.h>
// #include <forward_command_controller/forward_joint_group_command_controller.h>
#include <position_controllers/joint_group_position_controller.h>
#include <mh5_hardware/resource_interfaces.hpp>
#include <mh5_msgs/ActivateJoint.h>

#pragma once

namespace mh5_controllers
{


class DynamixelJointController : public controller_interface::Controller<mh5_hardware::DynamixelJointControlInterface>
{

public:
    DynamixelJointController()
    : controller_interface::Controller<mh5_hardware::DynamixelJointControlInterface> ()
    {}

    ~DynamixelJointController() {torque_srv_.shutdown(); reboot_srv_.shutdown(); }

    bool init(mh5_hardware::DynamixelJointControlInterface* hw, ros::NodeHandle &n);

    void starting(const ros::Time& /*time*/);

    void stopping(const ros::Time& /*time*/);

    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);


private:

    std::map<std::string, mh5_hardware::DynamixelJointControlHandle>                  joints_;
    std::map<std::string, std::vector< mh5_hardware::DynamixelJointControlHandle >>   groups_;
    std::string nn_;       // node name; for messages

    /**
     * @brief Holds torque activation commands to be processed during the 
     * update() processings.
     * The service callbacks only store "true" or "false" in this buffer
     * depending on the command processed.
     */
    realtime_tools::RealtimeBuffer<mh5_msgs::ActivateJoint::Request> torque_commands_buffer_;

    /**
     * @brief Holds reboot commands to be processed during the 
     * update() processings.
     * The service callbacks only store "true" or "false" in this buffer
     * depending on the command processed.
     */
    realtime_tools::RealtimeBuffer<mh5_msgs::ActivateJoint::Request> reboot_commands_buffer_;

    /**
     * @brief ROS Service that responds to the "switch_torque" calls.
     */
    ros::ServiceServer torque_srv_;

    /**
     * @brief ROS Service that responds to the "reboot" calls.
     */
    ros::ServiceServer reboot_srv_;

    /**
     * @brief Callback for processing "switch_torque" calls. Checks if the requested
     * group exists or if there is a joint by that name
     * 
     * @param req the service request; group/joint name  + desired state
     * @param res the service response; if things are successful + detailed message
     * @return true always
     */
    bool torqueCB(mh5_msgs::ActivateJoint::Request &req, mh5_msgs::ActivateJoint::Response &res);

   /**
     * @brief Callback for processing "reboot" calls. Checks if the requested
     * group exists or if there is a joint by that name
     * 
     * @param req the service request; group/joint name  + desired state
     * @param res the service response; if things are successful + detailed message
     * @return true always
     */
    bool rebootCB(mh5_msgs::ActivateJoint::Request &req, mh5_msgs::ActivateJoint::Response &res);
    
};


class  DynamixelPositionController : public controller_interface::MultiInterfaceController<
        hardware_interface::PositionJointInterface,
        mh5_hardware::DynamixelJointControlInterface>

// public position_controllers::JointGroupPositionController, DynamixelJointController
{
public:

    /**
     * @brief Construct a new Active Joint Controller object using a 
     * mh5_hardware::ActiveJointInterface interface.
     */
    DynamixelPositionController()
    :     controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, mh5_hardware::DynamixelJointControlInterface> (true)
    // position_controllers::JointGroupPositionController(), DynamixelJointController()
    {}
    
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    void starting(const ros::Time& /*time*/);

    void stopping(const ros::Time& /*time*/);

    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

    // bool initRequest(hardware_interface::RobotHW* robot_hw,
    //                 ros::NodeHandle&             root_nh,
    //                 ros::NodeHandle&             controller_nh,
    //                 ClaimedResources&            claimed_resources) override;

private:

    position_controllers::JointGroupPositionController*     pos_controller_;
    mh5_controllers::DynamixelJointController*              ctrl_controller_;

};

} // namespace