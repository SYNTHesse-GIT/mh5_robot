/**
 * @file foot_sensor.cpp
 * @author your name (you@domain.com)
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

#include "mh5_hardware/foot_sensor.hpp"

using namespace mh5_hardware;


void FootSensor::fromParam(ros::NodeHandle& nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph )
{
    DynamixelDevice::fromParam(nh, name, port, ph);

    // if(present_) {
    // }

    // setup hardware handles
    volt_curr_handle_ = mh5_hardware::VoltCurrHandle(name_, &voltage_, &current_);
}


void FootSensor::initRegisters()
{
    // writeRegister(9, 1, 0, TRIES);       // return delay
    // writeRegister(11, 1, 3, TRIES);      // operating mode
    // writeRegister(31, 1, 75, TRIES);     // temperature limit
    // writeRegister(32, 2, 135, TRIES);    // max voltage
    // writeRegister(44, 4, 1023, TRIES);   // velocity limit
    // writeRegister(48, 4, 4095, TRIES);   // max position
    // writeRegister(52, 4, 0, TRIES);      // min position

    // // direction
    // if (inverse_)
    //     writeRegister(10, 1, 5, TRIES);  // inverse; time profile
    // else
    //     writeRegister(10, 1, 4, TRIES);  // direct; time profile

    // // PID and FF
    // writeRegister(80, 2, 4000, TRIES);      // Position D Gain
    // writeRegister(82, 2, 0, TRIES);         // Position I Gain
    // writeRegister(84, 2, 1280, TRIES);       // Position P Gain
    // writeRegister(88, 2, 0, TRIES);         // FF 2nd Gain
    // writeRegister(90, 2, 0, TRIES);         // FF 1st Gain

    // // initilizes the active members to avoid issues later when the syncs start
    // active_command_ = 0.0;
    // active_state_ = 0.0;
    // active_command_flag_ = false;
    reboot_command_ = false;
}

bool FootSensor::read4Sensors(u_int16_t address, FootReading& readings)
{
    int     dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error       = 0;                        // Dynamixel error
    bool    result          = false;
    uint8_t buffer[8]       = {0};

    dxl_comm_result = ph_->readTxRx(port_, id_, address, 8, buffer, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        return false;
    } else {
        readings.fl = DXL_MAKEWORD(buffer[0], buffer[1]);
        readings.fr = DXL_MAKEWORD(buffer[2], buffer[3]);
        readings.bl = DXL_MAKEWORD(buffer[4], buffer[5]);
        readings.br = DXL_MAKEWORD(buffer[6], buffer[7]);
        return true;
    }
}

bool FootSensor::readCalibrationFactors()
{
    return true;
}
bool FootSensor::updateCalibrationFactors()
{
    return true;
}

bool FootSensor::readPower()
{
    return true;
}