#include <pluginlib/class_list_macros.hpp>
#include "mh5_controllers/dynamixel_joint_controller.hpp"
// #include "mh5_controllers/ActivateJoint.h"

namespace mh5_controllers
{

bool DynamixelJointController::init(mh5_hardware::DynamixelJointControlInterface* hw, ros::NodeHandle &n)
{
    if(!forward_command_controller::ForwardJointGroupCommandController<mh5_hardware::DynamixelJointControlInterface>::init(hw, n))
        return false;

    if(!n.hasParam("groups"))
    {
        ROS_INFO("[%s] no groups specified; all joints will be placed in a group called 'all'",
                  n.getNamespace().c_str());
        groups_["all"];
        for (auto & joint_name : hw->getNames()) {
            groups_["all"].push_back(hw->getHandle(joint_name));
        }
        ROS_INFO("[%s] group 'all' registered with %d items",
                  n.getNamespace().c_str(), groups_["all"].size());
    }
    else
    {
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
                    groups_[group].push_back(hw->getHandle(name));
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

    // a little ugly
    for (auto & handle : joints_) {
        if (handle.getName() == req.name) {
            torque_commands_buffer_.writeFromNonRT(req);
            res.success = true;
            res.message = "Joint " + req.name + " torque change buffered";
            return true;
        }
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

    // a little ugly
    for (auto & handle : joints_) {
        if (handle.getName() == req.name) {
            reboot_commands_buffer_.writeFromNonRT(req);
            res.success = true;
            res.message = "Joint " + req.name + " reboot buffered";
            return true;
        }
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

        for (auto & handle : joints_)
            if (handle.getName() == command.name)
            {
                ((mh5_hardware::DynamixelJointControlHandle&)handle).setActive(command.state);
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

        for (auto & handle : joints_)
            if (handle.getName() == command.name)
            {
                ((mh5_hardware::DynamixelJointControlHandle&)handle).setReboot(command.state);
                // we need to rest it because it would be latched
                reboot_commands_buffer_.initRT(mh5_msgs::ActivateJoint::Request());
                return;
            }
    }
}


/* activates torque for all joints */
// void DynamixelJointController::starting(const ros::Time& /*time*/)
// {
//     for (auto & handle : joints_) {
//          ((mh5_hardware::DynamixelJointControlHandle&)handle).setActive(true);
//     }
// }


// void DynamixelJointController::stopping(const ros::Time& /*time*/)
// {
//     for (auto & handle: joints_) {
//          ((mh5_hardware::DynamixelJointControlHandle&)handle).setActive(false);
//     }
// }



} // namespace

PLUGINLIB_EXPORT_CLASS(mh5_controllers::DynamixelJointController, controller_interface::ControllerBase)