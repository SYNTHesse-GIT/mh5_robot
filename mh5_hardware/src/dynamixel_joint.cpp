/**
 * @file dynamixel_joint.cpp
 * @author Alex Sonea
 * @brief 
 * @version 0.1
 * @date 2021-05-26
 * 
 * @copyright Copyright (c) 2021
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "mh5_hardware/dynamixel_joint.hpp"

using namespace mh5_hardware;

#define TRIES 5         // retry for dynamixel communication

void Joint::fromParam(ros::NodeHandle& nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph )
{
    DynamixelDevice::fromParam(nh, name, port, ph);

    if(present_) {
        inverse_ = nh.param<bool>(name + "/inverse", false);
        offset_ = nh.param<double>(name + "/offset", 0);
        profile_ = nh.param<int>(name + "/profile", 0);   // default is velocity profile
    }

    // setup hardware handles
    jointStateHandle_ = hardware_interface::JointStateHandle(name_, &position_state_, &velocity_state_, &effort_state_);
    jointStatusHandle_ = mh5_hardware::DynamixelStatusHandle(name_, &temperature_state_, &voltage_state_, &active_state_, &hwerr_state_);
    // jointPositionHandle_ = hardware_interface::JointHandle(jointStateHandle_, &position_command_);
    // jointPosVelHandle_ = hardware_interface::PosVelJointHandle(jointStateHandle_, &position_command_, &velocity_command_);
    jointControlHandle_ = mh5_hardware::DynamixelJointControlHandle (jointStateHandle_, &position_command_, &velocity_command_, &acceleration_command_, &active_command_, &reboot_command_);
    // jointTempVoltHandle_ = mh5_hardware::TempVoltHandle(name_, &temperature_state_, &voltage_state_);
}


bool Joint::isActive(bool refresh)
{
    if (refresh) {
        long value;
        if(!readRegister(64, 1, value, TRIES)) {
            ROS_ERROR("[%s] failed to read torque status for %s [%d]", nss_, name_.c_str(), id_);
        }
        else {
            active_state_ = (bool)value;
        }
    }
    return active_state_;
}



bool Joint::changeTorque(bool new_state)
{
    if (new_state) {
        // for torque activation make sure that the target position is the same
        // as the current position otherwise we risk a very sudden move of the
        // servo to a previous saved taget position
        long curr_pos;
        if (!readRegister(132, 4, curr_pos, TRIES)) {
            ROS_ERROR("[%s] failed to read current position for %s [%d]", nss_, name_.c_str(), id_);
            return false;
        }
        if (!writeRegister(116, 4, curr_pos, TRIES)) {
            ROS_ERROR("[%s] failed to seed target position for %s [%d]", nss_, name_.c_str(), id_);
            return false;
        }
    }
    if (writeRegister(64, 1, (int)new_state, TRIES)) {
        active_state_ = new_state;
        return true;
    }
    return false;
}


// bool Joint::toggleTorque()
// {
//     if(writeRegister(64, 1, (long)active_command_, TRIES)) {
//         active_state_ = active_command_;
//         return true;
//     }
//     else
//         return false;
// }


void Joint::initRegisters()
{
    DynamixelDevice::initRegisters();

    // profile
    writeRegister(10, 1, profile_*4 + (int)inverse_, TRIES);

    // initilizes the active members to avoid issues later when the syncs start
    active_command_ = false;
    active_state_ = false;
    reboot_command_ = false;
}