/**
 * @file dynamixel_device.cpp
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

#include "mh5_hardware/dynamixel_device.hpp"


using namespace mh5_hardware;


void DynamixelDevice::fromParam(ros::NodeHandle& nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph )
{
    name_ = name;
    port_ = port;
    ph_ = ph;
    nh_ = nh;
    nss_ = nh.getNamespace().c_str();

    int device_id;                    // stores from param server
    if (!nh.getParam(name + "/id", device_id)) {
        ROS_ERROR("[%s] ID not found for %s; will be disabled", nss_, name_.c_str());
        present_ = false;
    }
    else {
        id_ = (uint8_t)device_id;
        present_ = true;
    }
    if(!nh.getParam(name + "/inits", inits_)) {
        ROS_WARN("[%s] joint %s [%d] has not inits specified",  nss_, name_.c_str(), id_);
    }
    reboot_command_ = false;
}

void DynamixelDevice::initRegisters()
{
    for (auto init: inits_) {
        std::vector<std::string>    registers;
        if(!nh_.getParamCached("/dynamixel_inits/" + init, registers)) {
            ROS_ERROR("[%s] init %s used by %d does not exist", nss_, init.c_str(), id_);
            continue;
        }
        for (auto reg: registers) {
            std::vector<int>    sequence;
            if(!nh_.getParamCached("/dynamixel_inits/" + reg, sequence)) {
                ROS_ERROR("[%s] register init %s used by %d does not exist", nss_, reg.c_str(), id_);
                continue;
            }
            if (sequence.size() < 3) {
                ROS_ERROR("[%s] register init %s used by %d has less than 3 parameters", nss_, reg.c_str(), id_);
                continue;
            }
            if(!writeRegister(sequence[0], sequence[1], sequence[2], 5)) {
                 ROS_ERROR("[%s] register init %s used by %d failed", nss_, reg.c_str(), id_);
            }
        }
    }
}

bool DynamixelDevice::ping(const int num_tries )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool device_ok = false;
    
    for (int n=0; n < num_tries; n++)
    {
        dxl_comm_result = ph_->ping(port_, id_, &dxl_error);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_WARN("[%s] failed to communicate with device %s [%d]: %s", 
                      nss_, name_.c_str(), id_, ph_->getTxRxResult(dxl_comm_result));
            continue;
        }
        
        if (dxl_error != 0) {
            ROS_WARN("[%s] error reported when communicating with device %s [%d]: %s", 
                      nss_, name_.c_str(), id_, ph_->getRxPacketError(dxl_error));
            continue;
        }
        device_ok = true;
        break;
    }
    return device_ok;
}


bool DynamixelDevice::readRegister(const uint16_t address, const int size, long& value, const int num_tries)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;
    long buff = 0;                                  // buffer for reading value

    // we'll make several attempt in case there are communication errors
    for (int n=0; n < num_tries; n++)
    {
        switch(size) {
            case 1:
                dxl_comm_result = ph_->read1ByteTxRx(port_, id_, address, (uint8_t *)&buff, &dxl_error);
                break;
            case 2:
                dxl_comm_result = ph_->read2ByteTxRx(port_, id_, address, (uint16_t*) &buff, &dxl_error);
                break;
            case 4:
                dxl_comm_result = ph_->read4ByteTxRx(port_, id_, address, (uint32_t*) &buff, &dxl_error);
                break;
            default: {
                ROS_ERROR("[%s] Incorrect 'writeRegister' call with size %d", nss_, size);
                return false;
            }
        }

        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_WARN("[%s] readRegister communication failure (%s) for device %s [%d], register %d",
                      nss_, ph_->getTxRxResult(dxl_comm_result), name_.c_str(), id_, address);
            continue;
        }

        if (dxl_error != 0) {
            ROS_WARN("[%s] readRegister packet error (%s) for device %s [%d], register %d",
                      nss_, ph_->getRxPacketError(dxl_error), name_.c_str(), id_, address);
            continue;
        }

        result = true;
        value = buff;
    }

    return result;
}


bool DynamixelDevice::writeRegister(const uint16_t address, const int size, const long value, const int num_tries)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    // we'll make several attempt in case there are communication errors
    for (int n=0; n < num_tries; n++)
    {
        switch(size) {
            case 1:
                dxl_comm_result = ph_->write1ByteTxRx(port_, id_, address, (uint8_t) value, &dxl_error);
                break;
            case 2:
                dxl_comm_result = ph_->write2ByteTxRx(port_, id_, address, (uint16_t) value, &dxl_error);
                break;
            case 4:
                dxl_comm_result = ph_->write4ByteTxRx(port_, id_, address, (uint32_t) value, &dxl_error);
                break;
            default: {
                ROS_ERROR("[%s] incorrect 'writeRegister' call with size %d", nss_, size);
                return false;
            }
        }

        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR_ONCE("[%s] writeRegister communication failure (%s) for servo %s [%d], register %d",
                      nss_, ph_->getTxRxResult(dxl_comm_result), name_.c_str(), id_, address);
            continue;
        }

        if (dxl_error != 0) {
            ROS_ERROR_ONCE("[%s] writeRegister packet error (%s) for servo %s [%d], register %d",
                      nss_, ph_->getRxPacketError(dxl_error), name_.c_str(), id_, address);
            continue;
        }

        result = true;
        break;
    }

    return result;
}


bool DynamixelDevice::reboot(const int num_tries)
{
    //  it will reset the reboot command anyway
    reboot_command_ = false;
    for (int n=0; n < num_tries; n++)
    {
        int dxl_comm_result = COMM_TX_FAIL;             // Communication result
        uint8_t dxl_error = 0;                          // Dynamixel error

        dxl_comm_result = ph_->reboot(port_, id_, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            // ROS_ERROR_ONCE("Failed to reset device %s [%d]: %s", 
            //         name_.c_str(), id_, ph_->getTxRxResult(dxl_comm_result));
            continue;
        }
        else if (dxl_error != 0) {
            // ROS_ERROR_ONCE("Failed to reset device %s [%d]: %s", 
            //         name_.c_str(), id_, ph_->getRxPacketError(dxl_error));
            continue;
        }
        else {
            ROS_INFO("Successful rebooted device %s [%d]",  name_.c_str(), id_);
            return true;
        }
    }
    ROS_ERROR("Failed to reboot device %s [%d]: %s", name_.c_str(), id_);
    return false;
}
