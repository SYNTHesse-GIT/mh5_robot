#include <hardware_interface/joint_command_interface.h>
//#include <hardware_interface/internal/hardware_resource_manager.h>

#pragma once


namespace mh5_hardware
{


class DynamixelStatusHandle
{
private:
    std::string   name_;
    const double* temp_           = {nullptr};
    const double* volt_           = {nullptr};
    const bool*   activ_          = {nullptr};
    const int*    hwerr_          = {nullptr};

public:
    DynamixelStatusHandle() = default;

    DynamixelStatusHandle(const std::string& name, const double* temp, const double* volt, const bool* activ, const int* hwerr)
        : name_(name), temp_(temp), volt_(volt), activ_(activ), hwerr_(hwerr)
    {
        if (!temp)
        {
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Temperature data pointer is null.");
        }
        if (!volt)
        {
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Voltage data pointer is null.");
        }
        if (!activ)
        {
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Activation status data pointer is null.");
        }
        if (!hwerr)
        {
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Hardware error data pointer is null.");
        }
    }
    std::string getName() const { return name_; }
    double getTemperature() const { assert(temp_); return *temp_; }
    double getVoltage() const { assert(volt_); return *volt_; }
    bool isActive() const { assert(activ_); return *activ_; }
    int getHWError() const {assert(hwerr_); return *hwerr_; }

    const double* getTemperaturePtr() const { return temp_; }
    const double* getVoltagePtr() const { return volt_; }
    const bool* isActivePtr() const { return activ_; }
    const int* getHWErrorPtr() const { return hwerr_; }
};

class DynamixelStatusInterface : public hardware_interface::HardwareResourceManager<DynamixelStatusHandle> {};


class DynamixelJointControlHandle : public hardware_interface::JointHandle
{
private:
    // std::string   name_;
    // const double* position_           = {nullptr};
    // const double* velocity_           = {nullptr};
    bool*   active_          = {nullptr};
    bool*   reboot_          = {nullptr};

public:
    DynamixelJointControlHandle() = default;

    DynamixelJointControlHandle(const hardware_interface::JointStateHandle& js, double* pos_cmd, bool* activ, bool* reboot)
        : hardware_interface::JointHandle(js, pos_cmd), active_(activ), reboot_(reboot)
    {
        if (!activ)
        {
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Torque activation data pointer is null.");
        }
        if (!reboot)
        {
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Reboot command data pointer is null.");
        }
    }
    // std::string getName() const { return name_; }
    // void setCommand(double command) {assert(position_); *position_ = command;}
    // double getCommand() const {assert(position_); return *position_;}
    // const double* getCommandPtr() const {assert(position_); return position_;}
    void setActive(bool active) {assert(active_); *active_ = active;}
    bool getActiveCmd() const {assert(active_); return *active_;}
    const bool* getActiveCmdPtr() const {assert(active_); return active_;}
    void setReboot(bool reboot) {assert(reboot_); *reboot_ = reboot;}
    bool getRebootCmd() const {assert(reboot_); return *reboot_;}
    const bool* getRebootCmdPtr() const {assert(reboot_); return reboot_;}
};


class DynamixelJointControlInterface : public hardware_interface::HardwareResourceManager<DynamixelJointControlHandle, hardware_interface::ClaimResources> {};

/**
 * @brief Extends the hardware_interface::JointHandle with a boolean flag
 * that indicates when a new command was posted. This helps the HW interface
 * decide if that value needs to be replicated to the servos or not.
 */
class JointHandleWithFlag : public hardware_interface::JointHandle
{
public:

    JointHandleWithFlag() = default;

    /**
     * @brief Construct a new JointHandleWithFlag object by extending the
     * hardware_interface::JointHandle with an additional boolean
     * flag that indicates a new command has been issued.
     * 
     * @param js the JointStateHandle that is commanded
     * @param cmd pointer to the command attribute in the HW interface
     * @param cmd_flag pointed to the bool flag in the HW interface that is
     * used to indicate that the value was changed and therefore needs to be
     * synchronized by the HW.
     */
    JointHandleWithFlag(const JointStateHandle& js, double* cmd, bool* cmd_flag)
    : hardware_interface::JointHandle(js, (cmd)),
      cmd_flag_(cmd_flag)
      {
          if (!cmd_flag_)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command flag pointer is null.");
      }

    /**
     * @brief Overrides the hardware_interface::JointHandle setCommand()
     * method by setting the flag in the HW to true to indicate that a new
     * value was storred and therefore it needs to be synchronised after
     * calling the inherited method.
     * 
     * @param command the command set to the joint
     */
    void setCommand(double command) 
    {
        hardware_interface::JointHandle::setCommand(command);
        assert(cmd_flag_);
        *cmd_flag_ = true;
    }

private:

    /**
     * @brief Keeps the pointed to the flag in the HW that indicates when
     * value change.
     */
    bool* cmd_flag_ = {nullptr};

};

// class JointTorqueAndReboot : public JointHandleWithFlag
// {
// public:
//     JointTorqueAndReboot() = default;

//     JointTorqueAndReboot(const JointStateHandle& js, double* torque, bool* torque_flag, bool* reboot_flag)
//     : JointHandleWithFlag(js, torque, torque_flag), reboot_flag_(reboot_flag) 
//     {
//         if (!reboot_flag_)
//             throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Rebbot flag pointer is null.");
//     }

//     void setReboot(bool reboot) { assert(reboot_flag_); *reboot_flag_ = reboot; }
//     bool getReboot() { assert(reboot_flag_); return *reboot_flag_; }

// private:

//     bool* reboot_flag_ = {nullptr};

// };

// /**
//  * @brief Joint that supports activation / deactivation
//  * 
//  * To keep track of updates to the HW resource we use and additional flag
//  * that is set to true when a new command is issued to the servo. The
//  * communication loops will use this flag to determine which servos really
//  * need to be syncronised and will reset it once the synchronisation is
//  * finished.
//  */
// class ActiveJointInterface : public hardware_interface::HardwareResourceManager<JointTorqueAndReboot> {};


// class TempVoltHandle
// {
// public:

//     TempVoltHandle() = default;

//     TempVoltHandle(const std::string& name, const double* temp, const double* volt)
//         : name_(name), temp_(temp), volt_(volt)
//     {
//         if (!temp)
//         {
//         throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Temperature data pointer is null.");
//         }
//         if (!volt)
//         {
//         throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Voltage data pointer is null.");
//         }
//     }

//     std::string getName() const {return name_;}
//     double getTemperature()  const {assert(temp_); return *temp_;}
//     double getVoltage()  const {assert(volt_); return *volt_;}
//     const double* getTemperaturePtr() const {return temp_;}
//     const double* getVoltagePtr() const {return volt_;}

// private:

//     std::string name_;
//     const double* temp_           = {nullptr};
//     const double* volt_           = {nullptr};

// };

// class TempVoltInterface : public hardware_interface::HardwareResourceManager<TempVoltHandle> {};


class VoltCurrHandle
{
public:

    VoltCurrHandle() = default;

    VoltCurrHandle(const std::string& name, const double* volt, const double* curr)
        : name_(name), volt_(volt), curr_(curr)
    {
        if (!volt)
        {
        throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Voltage data pointer is null.");
        }
        if (!curr)
        {
        throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Current data pointer is null.");
        }
    }

    std::string getName() const {return name_;}
    double getVoltage()  const {assert(volt_); return *volt_;}
    double getCurrent()  const {assert(curr_); return *curr_;}
    const double* getVoltagePtr() const {return volt_;}
    const double* getCurrPtr() const {return curr_;}

private:

    std::string name_;
    const double* volt_           = {nullptr};
    const double* curr_           = {nullptr};

};

class VoltCurrInterface : public hardware_interface::HardwareResourceManager<VoltCurrHandle> {};

} // namespace
