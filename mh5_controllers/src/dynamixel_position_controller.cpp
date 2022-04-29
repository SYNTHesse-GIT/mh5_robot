#include <pluginlib/class_list_macros.hpp>
#include "mh5_controllers/dynamixel_position_controller.hpp"
// #include "mh5_controllers/ActivateJoint.h"

namespace mh5_controllers
{

bool DynamixelJointController::init(mh5_hardware::DynamixelJointControlInterface* hw, ros::NodeHandle &n)
{
    if(!controller_interface::Controller<mh5_hardware::DynamixelJointControlInterface>::init(hw, n))
        return false;

    // List of controlled joints
    std::string param_name = "joints";
    std::vector< std::string > joint_names;
    if(!n.getParam(param_name, joint_names))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }

    unsigned int n_joints = joint_names.size();
    if(n_joints == 0) {
      ROS_ERROR("List of joint names is empty.");
      return false;
    }

    for(unsigned int i=0; i<n_joints; i++) {
      try {
        joints_[joint_names[i]] = hw->getHandle(joint_names[i]);
      }
      catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }

    if(!n.hasParam("groups"))
    {
        ROS_INFO("[%s] no groups specified; all joints will be placed in a group called 'all'",
                  n.getNamespace().c_str());
        groups_["all"];
        for (auto & joint : joints_) {
            groups_["all"].push_back(joint.second);
        }
        ROS_INFO("[%s] group 'all' registered with %d items",
                  n.getNamespace().c_str(), groups_["all"].size());
    }
    else {
        std::vector<std::string> groups;
        n.getParam("groups", groups);
        for (auto & group : groups)
        {
            std::vector<std::string>    names;      // could be joints or subgroups
            std::vector<std::string>    joint_names;// only joint names
            std::vector<mh5_hardware::DynamixelJointControlHandle>    joint_handles;
            groups_[group];
            n.getParam(group, names);
            for (auto & name : names) {
                if (groups_.count(name))
                {
                    // handle subgroup; we copy the handles from the subgroup
                    for (auto & handle : groups_[name]) {
                        groups_[group].push_back(handle);
                    }
                }
                else {
                    groups_[group].push_back(joints_[name]);
                }
            }
            ROS_INFO("[%s] group '%s' registered with %d items",
                  n.getNamespace().c_str(), group.c_str(), groups_[group].size());
        }
    }
    // we need this bacuse we're reusing the ardware_interface::JointCommandInterface
    // in mh5_hardware::ActiveJointInterface and this uses registered handles
    // since the effective change of the torque is done in the hardware
    // interface (MH5DynamixelInterface) in the write() method that is entirely
    // under out control so no coflicts can arise between updating torque status
    // registers and other write steps.
    // hw->clearClaims();

    torque_srv_ = n.advertiseService("torque", &DynamixelJointController::torqueCB, this);
    reboot_srv_ = n.advertiseService("reboot", &DynamixelJointController::rebootCB, this);

    return true;
}


bool DynamixelJointController::torqueCB(mh5_msgs::ActivateJoint::Request &req, mh5_msgs::ActivateJoint::Response &res)
{
    if (groups_.count(req.name))  {
        torque_commands_buffer_.writeFromNonRT(req);
        res.success = true;
        res.message = "Group " + req.name + " torque change buffered";
        return true;
    }

    if (joints_.find(req.name) != joints_.end()) {
        torque_commands_buffer_.writeFromNonRT(req);
        res.success = true;
        res.message = "Joint " + req.name + " torque change buffered";
        return true;
    }

    // no group nor joint with that name available
    res.success = false;
    res.message = "No group or joint named " + req.name + " found";
    return false;
}


bool DynamixelJointController::rebootCB(mh5_msgs::ActivateJoint::Request &req, mh5_msgs::ActivateJoint::Response &res)
{
    if (groups_.count(req.name))  {
        reboot_commands_buffer_.writeFromNonRT(req);
        res.success = true;
        res.message = "Group " + req.name + " reboot buffered";
        return true;
    }

    if (joints_.find(req.name) != joints_.end()) {
        reboot_commands_buffer_.writeFromNonRT(req);
        res.success = true;
        res.message = "Joint " + req.name + " reboot buffered";
        return true;
    }

    // no group nor joint with that name available
    res.success = false;
    res.message = "No group or joint named " + req.name + " found";
    return false;
}



void DynamixelJointController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
    mh5_msgs::ActivateJoint::Request command = *torque_commands_buffer_.readFromRT();

    if (command.name != "")
    {
        if (groups_.count(command.name))  {
            for (auto & handle : groups_[command.name])
                handle.setActive(command.state);
            // we need to reset it because it would be latched
            torque_commands_buffer_.initRT(mh5_msgs::ActivateJoint::Request());
            return;
        }

        if (joints_.find(command.name) != joints_.end()) {
            joints_[command.name].setActive(command.state);
            // we need to rest it because it would be latched
            torque_commands_buffer_.initRT(mh5_msgs::ActivateJoint::Request());
            return;
        }
    }

    command = *reboot_commands_buffer_.readFromRT();

    if (command.name != "")
    {
        if (groups_.count(command.name))  {
            for (auto & handle : groups_[command.name])
                handle.setReboot(command.state);
            // we need to reset it because it would be latched
            reboot_commands_buffer_.initRT(mh5_msgs::ActivateJoint::Request());
            return;
        }

        if (joints_.find(command.name) != joints_.end())
        {
            joints_[command.name].setReboot(command.state);
            // we need to rest it because it would be latched
            reboot_commands_buffer_.initRT(mh5_msgs::ActivateJoint::Request());
            return;
        }
    }
}


/* activates torque for all joints */
void DynamixelJointController::starting(const ros::Time& /*time*/)
{
    ROS_INFO("Activating joints...");
    for (auto & joint : joints_) {
         joint.second.setActive(true);
    }
}


void DynamixelJointController::stopping(const ros::Time& /*time*/)
{
    for (auto & joint: joints_) {
         joint.second.setActive(false);
    }
    ROS_INFO("Joints deactivated...");
}


bool DynamixelPositionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
    hardware_interface::PositionJointInterface*   pos_hw = robot_hw->get<hardware_interface::PositionJointInterface>();
    if(!pos_hw) {
        ROS_ERROR("Requires PositionJointInterface");
        return false;
    }
    pos_controller_ = new position_controllers::JointGroupPositionController();
    if(! pos_controller_->init(pos_hw, controller_nh)) {
        ROS_ERROR("Failed to initialize the JointGroupPositionController controller object");
        return false;
    }
    pos_hw->clearClaims();

    mh5_hardware::DynamixelJointControlInterface* ctrl_hw = robot_hw->get<mh5_hardware::DynamixelJointControlInterface>();
    if(!ctrl_hw) {
        ROS_ERROR("Requires DynamixelJointControlInterface");
        return false;
    }
    ctrl_controller_ = new DynamixelJointController();
    if(! ctrl_controller_->init(ctrl_hw, controller_nh)) {
        ROS_ERROR("Failed to initialize the DynamixelJointController controller object");
        return false;
    }
    return true;
}


void DynamixelPositionController::starting(const ros::Time& time)
{
    pos_controller_->starting(time);
    ctrl_controller_->starting(time);
}


void DynamixelPositionController::stopping(const ros::Time& time)
{
    pos_controller_->stopping(time);
    ctrl_controller_->stopping(time);
}


void DynamixelPositionController::update(const ros::Time& time, const ros::Duration& period)
{
    pos_controller_->update(time, period);
    ctrl_controller_->update(time, period);
}


// bool DynamixelPositionController::initRequest(hardware_interface::RobotHW* robot_hw,
//                     ros::NodeHandle&             root_nh,
//                     ros::NodeHandle&             controller_nh,
//                     ClaimedResources&            claimed_resources)



} // namespace

PLUGINLIB_EXPORT_CLASS(mh5_controllers::DynamixelPositionController, controller_interface::ControllerBase)