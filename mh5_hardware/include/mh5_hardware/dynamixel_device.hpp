#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <mh5_hardware/port_handler.hpp>

#pragma once

namespace mh5_hardware
{

/**
 * @brief Represents a generic Dyanmixel device.
 * 
 */
class DynamixelDevice
{
public:

    /**
     * @brief Default constructor
     */
    DynamixelDevice() {}

    /**
     * @brief Uses information from the paramter server to initialize the Device.
     * 
     * It will look for the following paramters in the server, under the joint name:
     * 
     * - ``id``: the Dynamixel ID of the sensor; if missing the sensor will be marked 
     * as not present (ex. present_ = false) and this will exclude it from all
     * communication
     * 
     * @param hw_nh node handle to the hardware interface
     * @param name name given to this device
     * @param port Dynamixel port used for communication; should have been
     * checked and opened prior by the HW interface
     * @param ph Dynamixel port handler for communication; should have been
     * checked and initialized priod by the HW interface
     */
    void fromParam(ros::NodeHandle& hw_nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph);

    /**
     * @brief Returns the Dynamixel ID of the device
     * 
     * @return uint8_t the ID of the device.
     */
    uint8_t id() { return id_;}

    /**
     * @brief Returns the name of the device
     * 
     * @return std::string the name of the device.
     */
    std::string name() { return name_;}

    /**
     * @brief Returns if the device is present (all settings are ok and communication
     * with it was successfull).
     * 
     * @return true if the device is physically present
     * @return false if the device could not be detected
     */
    bool present() { return present_;}

    /**
     * @brief Updates the present flag of the device.
     * 
     * @param state the desired state (true == present, false = not present)
     */
    void setPresent(bool state) { present_ = state;}

    /**
     * @brief Performs a Dynamixel ping to the sensor. It will try up to num_tries
     * times in case there is no answer or there are communication errors.
     * 
     * @param num_tries how many tries to make if there are no answers
     * @return true if the joint has responded
     * @return false if the joint failed to respond after num_tries times
     */
    bool ping(const int num_tries);

    /**
     * @brief Hard-codes the initialization of the device. Subclasses must
     * override the method.
     * 
     */
    virtual void initRegisters() = 0;

    /**
     * @brief Convenience method for writing a register to the device. Depending
     * on the size parameter it will call write1ByteTxRx(), write2ByteTxRx() or
     * write4ByteTxRx().
     * 
     * @param address the address of the register to write to
     * @param size the size of the register to write to
     * @param value a value to write; it will be type casted to uint8_t, uint16_t
     * or unit32_t depending on the size parameter
     * @param num_tries number of times to try in case there are errors
     * @return true if the write was sucessful
     * @return false if there was a communication or hardware error
     */
    bool writeRegister(const uint16_t address, const int size, const long value, const int num_tries);

    /**
     * @brief Convenience method for reading a register from the device. Depending
     * on the size parameter it will call read1ByteTxRx(), read2ByteTxRx() or
     * read4ByteTxRx().
     * 
     * @param address the address of the register to read from
     * @param size the size of the register to read
     * @param value a value to store the read result; it will be type casted to uint8_t, uint16_t
     * or unit32_t depending on the size parameter
     * @param num_tries number of times to try in case there are errors
     * @return true if the read was sucessful
     * @return false if there was a communication or hardware error
     */
    bool readRegister(const uint16_t address, const int size, long& value, const int num_tries);


    /**
     * @brief Reboots the device by invoking the REBOOT Dynamixel instruction
     * 
     * @param num_tries how many tries to make if there are no answers
     * @return true if the reboot was successful
     * @return false if there were communication of harware errors
     */
    bool reboot(const int num_tries);


    /**
     * @brief Indicates if there was a command to reboot the device that
     * was not yet completed. It simply returns the reboot_command_flag_
     * member that should be set whenever a controllers wants to reboot
     * the device.
     * 
     * @return true there is a reset that was not syncronised to hardware
     * @return false there is no change in the status
     */
    bool shouldReboot() { return reboot_command_flag_; }


    /**
     * @brief Resets to false the reboot_command_flag_. Normally 
     * used by the sync loops after successful processing of an
     * update.
     */
    void resetRebootCommandFlag() { reboot_command_flag_ = 0.0; reboot_command_flag_ = false; }


protected:
    /// @brief The name of the device
    std::string                         name_;

    /// @brief The communication port to be used
    mh5_port_handler::PortHandlerMH5*   port_;

    /// @brief Dynamixel packet handler to be used
    dynamixel::PacketHandler*           ph_;

    /// @brief The node handler of the owner (hardware interface)
    ros::NodeHandle                     nh_;

    /// @brief Name of the owner as a c_str() - for easy printing of messages
    const char*                         nss_;

    //actual device
    /// @brief Device ID
    uint8_t         id_;

    /// @brief Device is present (true) or not (false)
    bool            present_;

    /// @brief Controller requested a reboot and is not yet syncronised
    bool            reboot_command_flag_;
};


} //namespace