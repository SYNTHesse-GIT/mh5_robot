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

    // writeRegister(9, 1, 0, TRIES);       // return delay
    // writeRegister(11, 1, 3, TRIES);      // operating mode = position control mode
    // writeRegister(31, 1, 75, TRIES);     // temperature limit
    // writeRegister(32, 2, 135, TRIES);    // max voltage
    // writeRegister(44, 4, 1023, TRIES);   // velocity limit
    // writeRegister(48, 4, 4095, TRIES);   // max position
    // writeRegister(52, 4, 0, TRIES);      // min position

    // direction
    if (inverse_)
        writeRegister(10, 1, 1, TRIES);  // inverse; velocity profile
    else
        writeRegister(10, 1, 0, TRIES);  // direct; velocity profile

    // PID and FF
    writeRegister(80, 2, 4000, TRIES);      // Position D Gain
    writeRegister(82, 2, 0, TRIES);         // Position I Gain
    writeRegister(84, 2, 200, TRIES);       // Position P Gain
    writeRegister(88, 2, 0, TRIES);         // FF 2nd Gain
    writeRegister(90, 2, 0, TRIES);         // FF 1st Gain
    // INDIRECT REGISTERS
    // writeRegister(168, 2, 64, TRIES);        // torque status (64)
    // writeRegister(170, 2, 70, TRIES);       // HW error (70)
    // writeRegister(172, 2, 144, TRIES);      // voltage L
    // writeRegister(174, 2, 145, TRIES);      // voltage H
    // writeRegister(176, 2, 146, TRIES);      // temperature
    // initilizes the active members to avoid issues later when the syncs start
    active_command_ = false;
    active_state_ = false;
    // active_command_flag_ = false;
    // reboot_command_ = false;
}