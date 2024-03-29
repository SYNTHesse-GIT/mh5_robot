/**
 * @file dynamixel_joint.hpp
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

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include "port_handler.hpp"
#include "resource_interfaces.hpp"
#include "dynamixel_device.hpp"

#pragma once

namespace mh5_hardware
{

/**
 * @brief Represents a Dynamixel servo with the registers and communication
 * methods.
 * 
 * Also has convenience methods for creating HW interfaces for access by
 * controllers.
 */
class Joint : public DynamixelDevice
{
public:

    /**
     * @brief Default constructor
     */
    Joint(): DynamixelDevice() {}

    /**
     * @brief Uses information from the paramter server to initialize the Joint.
     * 
     * It will look for the following paramters in the server, under the joint name:
     * 
     * - ``id``: the Dynamixel ID of the servo; if missing the joint will be marked 
     * as not prosent (ex. present_ = false) and this will exclude it from all
     * communication
     * 
     * - ``inverse``: indicates that the joint has position values specified CW (default)
     * are CCW see https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#drive-mode10
     * bit 0. If not present the default is ``false``
     * 
     * - ``offset``: a value [in radians] that will be added to converted raw position
     * from the hardware register to report present position of servos in radians.
     * Conversely it will be substracted from the desired command position before
     * converting to the raw position value to be stored in the servo.
     * 
     * Initializes the jointStateHandle_, jointPosVelHandle_ and jointActiveHandle_
     * attributes.
     * 
     * @param hw_nh node handle to the harware interface
     * @param name name given to this joint
     * @param port Dynamixel port used for communication; should have been
     * checked and opened prior by the HW interface
     * @param ph Dynamixel port handler for communication; should have been
     * checked and initialized priod by the HW interface
     */
    void fromParam(ros::NodeHandle& hw_nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph) override;

    /**
     * @brief Hard-codes the initialization of the following registers in the
     * joint (see https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table).
     * 
     * The registers are initialized as follows:
     * 
     * Register          | Address | Value | Comments                 
     * ----------------- | ------- | ----- | -------------------------
     * return delay      | 9       | 0     | 0 us delay time          
     * drive mode        | 10      | 4     | if no "inverse" mode set 
     * drive mode        | 10      | 5     | if "inverse" mode set    
     * operating mode    | 11      | 3     | position control mode    
     * temperature limit | 31      | 75    | 75 degrees Celsius       
     * max voltage       | 32      | 135   | 13.5 V                   
     * velocity limit    | 44      | 1023  | max velocity             
     * max position      | 48      | 4095  | max value                
     * min position      | 52      | 0     | min value                
     * 
     * Other registers might be added in the future.
     */
    void initRegisters() override;

    /**
     * @brief Returns if the joint is active (torque on).
     * 
     * @param refresh if this parameter is true it will force a re-read of the 
     * register 64 from the servo otherwise it will report the cached value
     * 
     * @return true the torque is active
     * @return false the torque is inactive
     */
    bool isActive(bool refresh = false);

    void setActive(bool active) { active_state_ = active; }

    int getHWError() { return hwerr_state_; }

    void setHWEror(int hwerr) { hwerr_state_ = hwerr; }

    bool getTorqueCommand() { return active_command_; }

    bool changeTorque(bool new_state);

    bool shouldChangeTorque() { return active_command_ != active_state_;}

    // /**
    //  * @brief Resets to false the active_command_flag_. Normally 
    //  * used by the sync loops after successful processing of an
    //  * update.
    //  */
    // void resetActiveCommandFlag() { active_command_flag_ = false;}

    // /**
    //  * @brief Changes the torque by writing into register 64 in the hardware
    //  * using the active_command_ value.
    //  * If the change is successfull it will reset the active_command_flag_.
    //  * 
    //  * @return true successful change
    //  * @return false communication or harware error
    //  */
    // bool toggleTorque();

    /**
     * @brief Produces an internal format for torque status based on a desired 
     * command.
     * 
     * @return uint8_t value suitable for writing to the hardware for the
     * desired torque status.
     */
    uint8_t getRawTorqueActiveFromCommand() { return (uint8_t) active_command_;}

    /**
     * @brief Set the position_state_ (represented in radians) from a raw_pos
     * that represents the value read from the hardware. It takes into account
     * the servo's charactistics, and the offset with the formula:
     * 
     *      position_state_ = (raw_pos - 2047 ) * 0.001533980787886 + offset_
     * 
     * @param raw_pos a raw position as read from the hardware; this will 
     * already contain the "inverse" classification.
     */
    void setPositionFromRaw(int32_t raw_pos) { position_state_ = (raw_pos - 2047 ) * 0.001533980787886 + offset_;}


    /**
     * @brief Set the velocity_state_ (represented in radians/sec) from a raw_vel
     * that represents the value read from the hardware. It takes into account
     * the servo's charactistics with the formula:
     * 
     *      velocity_state_ = raw_vel * 0.023980823922402
     * 
     * @param raw_vel a raw velocity as read from the hardware; this will 
     * already contain the "inverse" classification and is also signed
     */
    void setVelocityFromRaw(int32_t raw_vel) { velocity_state_ = raw_vel * 0.023980823922402;}

    /**
     * @brief Set the effort_state_ (represented in Nm) from a raw_eff
     * that represents the value read from the hardware. It takes into account
     * the servo's charactistics with the formula:
     * 
     *      effort_state_ = raw_eff * 0.0014
     * 
     * @param raw_eff a raw effort as read from the hardware; this will 
     * already contain the "inverse" classification and is also signed
     */
    void setEffortFromRaw(int32_t raw_eff) { effort_state_ = raw_eff * 0.0014;}

    /**
     * @brief @brief Set the voltage_state_ (represented in V) 
     * from a raw_volt that represents the value read from the hardware. The method
     * simply divides with 10 and converts to double.
     * 
     * @param raw_volt the value of voltage as read in hardware
     */
    void setVoltageFromRaw(int16_t raw_volt) { voltage_state_ = raw_volt / 10.0; }


    /**
     * @brief Set the temperature_state_ (represented in degrees Celsius) 
     * from a raw_temp that represents the value read from the hardware. The method
     * simply converts to double.
     * 
     * @param raw_temp 
     */
    void setTemperatureFromRaw(int8_t raw_temp) { temperature_state_ = (double) raw_temp; }

    /**
     * @brief Produces an internal format for position based on a desired 
     * command position (expressed in radians) using the formula:
     * 
     *      result = (position_command_ - offset_) / 0.001533980787886 + 2047
     * 
     * @return int32_t a value suitable for writing to the hardware for the
     * desired position in position_command_ expressed in radians.
     */
    int32_t getRawPositionFromCommand() {return (int32_t)((position_command_ - offset_) / 0.001533980787886 + 2047);}
    
    /**
     * @brief The velocity_command_ indicates the desired velocity (in rad/s) for
     * the execution of the position commands. Since we configure the servo in time
     * profile mode, the command is translated into a desired duration for the
     * execution of the position command, that is after that stored into 
     * register 112. For this the method calculates the delta between the desired position
     * and the current position divided by the desired velocity, obtaining thus
     * the desired duration for the move. The number is then multiplied with 1000
     * as the harware expect the duration in ms. The full formula for the value
     * is:
     * 
     *      result = abs(velocity_command_)[rad/s] * 2pi / 60 / 0.229
     * 
     * @return uint32_t a value suitable for writing to the hardware profile velocity
     * for the desired position in velocity_command_ expressed in radians/s.
     */
    // [rad/s] * 60 / 2pi -> [rev/min] /0.229 -> [raw]
    uint32_t getVelocityProfileFromCommand() { 
        if (profile_ == 0) {
            // velocity profile; velocity_command_ represents actual angular velocity
            return abs(velocity_command_) * 0.45729150707275;
        }
        else { // (profile_ == 1)
            // time profile; velocity_command_ represents the duration in seconds
            return abs(velocity_command_) * 1000;
        }
    }

    uint32_t getAccelerationProfileFromCOmmand() { 
        if (profile_ == 0) {
             // velocity profile;acceleration_command_ represents actual angular acceleration
            return abs(acceleration_command_) * 0.45729150707275;
        }
        else { //(profile_ == 1)
            // time profile; acceleration_command_ represents the duration in seconds
            return abs(acceleration_command_) * 1000;
        }
    }


    /**
     * @brief Returns the handle to the joint position interface object for this joint
     * 
     * @return const hardware_interface::JointStateHandle& 
     */
    const hardware_interface::JointStateHandle& getJointStateHandle() { return jointStateHandle_; }

    /**
     * @brief Returns the handle to the joint status interface object for this joint
     * 
     * @return const mh5_hardware::DynamixelStatusHandle& 
     */
    const mh5_hardware::DynamixelStatusHandle& getJointStatusHandle() { return jointStatusHandle_; }

    /**
     * @brief Returns the handle to the joint position control interface object 
     * 
     * @return const hardware_interface::JointStatusHandle& 
     */
    const hardware_interface::JointHandle& getJointPositionHandle() { return jointPositionHandle_; }

    /**
     * @brief Returns the handle to the joint position / velocity command interface object for this joint
     * 
     * @return const hardware_interface::PosVelJointHandle& 
     */
    const hardware_interface::PosVelJointHandle& getJointPosVelHandle() { return jointPosVelHandle_; }

    /**
     * @brief Returns the handle to the joint activation command interface object for this joint
     * 
     * @return const mh5_hardware::JointTorqueAndReboot& 
     */
    const mh5_hardware::DynamixelJointControlHandle& getJointControlHandle() { return jointControlHandle_; }


protected:
    // servo registers
    bool         inverse_;            /// @brief Servo uses inverse rotation
    double       offset_;             /// @brief Offest for servo from 0 position (center) in radians
    int          profile_;            /// @brief Profile mode (velocity or time see Drive Mode register 10)
    double       position_state_;     /// @brief Current position in radians
    double       velocity_state_;     /// @brief Current velocity in radians/s
    double       effort_state_;       /// @brief Current effort in Nm
    bool         active_state_;       /// @brief Current torque state [0.0 or 1.0]
    int          hwerr_state_;        /// @brief Hardware error code
    double       voltage_state_;      /// @brief Current voltage [V]
    double       temperature_state_;  /// @brief Current temperature deg C

    //commands
    double       position_command_;   /// @brief Desired position in radians
    double       velocity_command_;   /// @brief Desired velocity profile [rad/s]
    double       acceleration_command_; // @brief Desired acceleration profile [rad/s^2]
    // bool            poistion_command_flag_;
    bool         active_command_;     /// @brief Desired torque state [0.0 or 1.0]
    bool         reboot_command_;     /// @brief Reboot command indicator
    // bool            active_command_flag_;

    //hardware handles
    
    hardware_interface::JointStateHandle        jointStateHandle_;  /// @brief A handle that provides access to position, velocity and effort
    mh5_hardware::DynamixelStatusHandle         jointStatusHandle_; /// @brief A handle that provides access to temperature, voltage, activation status and hardware error
    hardware_interface::JointHandle             jointPositionHandle_; /// @brief A hadle for commanding the position of a joint
    mh5_hardware::DynamixelJointControlHandle   jointControlHandle_; /// @brief A handle that provides access to desired torque state

    hardware_interface::PosVelJointHandle   jointPosVelHandle_; /// @brief A handle that provides access to desired position and desired velocity


};


} //namespace