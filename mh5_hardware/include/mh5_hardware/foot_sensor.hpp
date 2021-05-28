/**
 * @file foot_sensor.hpp
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

#include "dynamixel_device.hpp"
#include "resource_interfaces.hpp"

#pragma once

namespace mh5_hardware
{

// register addresses
#define ADDR_RAW_SENSORS    70
#define ADDR_LPF_SENSORS    78
#define ADDR_CAL_SENSORS    100
#define ADDR_CAL_FACTORS    20
#define ADDR_POWER          87

typedef struct {
    u_int16_t   fl;     // front left
    u_int16_t   fr;     // front right
    u_int16_t   bl;     // back left
    u_int16_t   br;     // back right
} FootReading;


typedef struct {
    u_int16_t   fl_mul; // front left multiplier
    u_int16_t   fl_div; // front left divisor
    u_int16_t   fr_mul; // front right multiplier
    u_int16_t   fr_div; // front right divisor
    u_int16_t   bl_mul; // back left multiplier
    u_int16_t   bl_div; // back left divisor
    u_int16_t   br_mul; // back right multiplier
    u_int16_t   br_div; // back right divisor
} CalibrationFactors;

/**
 * @brief Represents a Dynamixel Foot sensor.
 * 
 * Also has convenience methods for creating HW interfaces for access by
 * controllers.
 */
class FootSensor: public mh5_hardware::DynamixelDevice
{
public:

    /**
     * @brief Default constructor
     */
    FootSensor() {}

    void fromParam(ros::NodeHandle& hw_nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph) override;
    
    /**
     * @brief Hard-codes the initialization of the registers in the
     * foot (see <link to the documentation>).
     * 
     * The registers are initialized as follows:
     * 
     * Register          | Address | Value | Comments                 
     * ----------------- | ------- | ----- | -------------------------
     * 
     * Other registers might be added in the future.
     */
    void initRegisters() override;


    bool readRawSensors() { return read4Sensors(ADDR_RAW_SENSORS, raw_readings_); }
    bool readLPFSensors() { return read4Sensors(ADDR_LPF_SENSORS, lpf_readings_); }
    bool readCalibratedSensors() { return read4Sensors(ADDR_CAL_SENSORS, foot_readings_); }


    bool readCalibrationFactors();
    bool updateCalibrationFactors();

    bool readPower();

    const mh5_hardware::VoltCurrHandle& getVoltCurrHandle() { return volt_curr_handle_; }
    // /**
    //  * @brief Returns the handle to the joint position interface object for this joint
    //  * 
    //  * @return const hardware_interface::JointStateHandle& 
    //  */
    // const hardware_interface::JointStateHandle& getJointStateHandle() { return jointStateHandle_; }

    // /**
    //  * @brief Returns the handle to the joint position / velocity command interface object for this joint
    //  * 
    //  * @return const hardware_interface::PosVelJointHandle& 
    //  */
    // const hardware_interface::PosVelJointHandle& getJointPosVelHandle() { return jointPosVelHandle_; }

    // /**
    //  * @brief Returns the handle to the joint activation command interface object for this joint
    //  * 
    //  * @return const mh5_hardware::JointTorqueAndReboot& 
    //  */
    // const mh5_hardware::JointTorqueAndReboot& getJointActiveHandle() { return jointActiveHandle_; }


protected:
    FootReading         foot_readings_;
    FootReading         raw_readings_;
    FootReading         lpf_readings_;

    CalibrationFactors  calibration_factors_;

    double              voltage_;
    double              current_;

    bool read4Sensors(u_int16_t address, FootReading& readings);

    mh5_hardware::VoltCurrHandle               volt_curr_handle_;
    // //hardware handles
    // /// @brief A handle that provides access to position, velocity and effort
    // hardware_interface::JointStateHandle    jointStateHandle_;

    // /// @brief A handle that provides access to desired position and desired velocity
    // hardware_interface::PosVelJointHandle   jointPosVelHandle_;

    // /// @brief A handle that provides access to desired torque state
    // mh5_hardware::JointTorqueAndReboot       jointActiveHandle_;
};


} //namespace