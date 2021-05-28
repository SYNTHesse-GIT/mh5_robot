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
    jointPosVelHandle_ = hardware_interface::PosVelJointHandle(jointStateHandle_, &position_command_, &velocity_command_);
    jointActiveHandle_ = mh5_hardware::JointTorqueAndReboot (jointStateHandle_, &active_command_, &active_command_flag_, &reboot_command_flag_);
    jointTempVoltHandle_ = mh5_hardware::TempVoltHandle(name_, &temperature_state_, &voltage_state_);
}


bool Joint::isActive(bool refresh)
{
    if (refresh) {
        long value;
        if(!readRegister(64, 1, value, TRIES)) {
            ROS_ERROR("[%s] failed to read torque status for %s [%d]", nss_, name_.c_str(), id_);
        }
        else {
            active_state_ = (double)value;
        }
    }
    return (active_state_ != 0);
}


bool Joint::torqueOn()
{
    return writeRegister(64, 1, 1, TRIES);
}


bool Joint::torqueOff()
{
    return writeRegister(64, 1, 0, TRIES);
}


bool Joint::toggleTorque()
{
    if(writeRegister(64, 1, (long)active_command_, TRIES)) {
        active_state_ = active_command_;
        return true;
    }
    else
        return false;
}


void Joint::initRegisters()
{
    writeRegister(9, 1, 0, TRIES);       // return delay
    writeRegister(11, 1, 3, TRIES);      // operating mode
    writeRegister(31, 1, 75, TRIES);     // temperature limit
    writeRegister(32, 2, 135, TRIES);    // max voltage
    writeRegister(44, 4, 1023, TRIES);   // velocity limit
    writeRegister(48, 4, 4095, TRIES);   // max position
    writeRegister(52, 4, 0, TRIES);      // min position

    // direction
    if (inverse_)
        writeRegister(10, 1, 5, TRIES);  // inverse; time profile
    else
        writeRegister(10, 1, 4, TRIES);  // direct; time profile

    // PID and FF
    writeRegister(80, 2, 4000, TRIES);      // Position D Gain
    writeRegister(82, 2, 0, TRIES);         // Position I Gain
    writeRegister(84, 2, 1280, TRIES);       // Position P Gain
    writeRegister(88, 2, 0, TRIES);         // FF 2nd Gain
    writeRegister(90, 2, 0, TRIES);         // FF 1st Gain

    // initilizes the active members to avoid issues later when the syncs start
    active_command_ = 0.0;
    active_state_ = 0.0;
    active_command_flag_ = false;
    reboot_command_flag_ = false;
}