// #include <hardware_interface/joint_command_interface.h>
//#include <hardware_interface/internal/hardware_resource_manager.h>

#pragma once


namespace mh5_hardware
{


class TempVoltHandle
{
public:

    TempVoltHandle() = default;

    TempVoltHandle(const std::string& name, const double* temp, const double* volt)
        : name_(name), temp_(temp), volt_(volt)
    {
        if (!temp)
        {
        throw HardwareInterfaceException("Cannot create handle '" + name + "'. Temperature data pointer is null.");
        }
        if (!volt)
        {
        throw HardwareInterfaceException("Cannot create handle '" + name + "'. Voltage data pointer is null.");
        }
    }

    std::string getName() const {return name_;}
    double getTemperature()  const {assert(pos_); return *temp_;}
    double getVoltage()  const {assert(vel_); return *volt_;}
    const double* getTemperaturePtr() const {return temp_;}
    const double* getVoltagePtr() const {return volt_;}

private:

    std::string name_;
    const double* temp_           = {nullptr};
    const double* volt_           = {nullptr};

};

class JointTorqueAndReboot : public JointHandleWithFlag
{
public:
    JointTorqueAndReboot() = default;

    JointTorqueAndReboot(const JointStateHandle& js, double* torque, bool* torque_flag, bool* reboot_flag)
    : JointHandleWithFlag(js, torque, torque_flag), reboot_flag_(reboot_flag) 
    {
        if (!reboot_flag_)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Rebbot flag pointer is null.");
    }

    void setReboot(bool reboot) { assert(reboot_flag_); *reboot_flag_ = reboot; }
    bool getReboot() { assert(reboot_flag_); return *reboot_flag_; }

private:

    bool* reboot_flag_ = {nullptr};

};

/**
 * @brief Joint that supports activation / deactivation
 * 
 * To keep track of updates to the HW resource we use and additional flag
 * that is set to true when a new command is issued to the servo. The
 * communication loops will use this flag to determine which servos really
 * need to be syncronised and will reset it once the synchronisation is
 * finished.
 */
class ActiveJointInterface : public hardware_interface::HardwareResourceManager<JointTorqueAndReboot> {};


}