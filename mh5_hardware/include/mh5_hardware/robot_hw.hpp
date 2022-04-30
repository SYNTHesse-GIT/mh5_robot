#include <combined_robot_hw/combined_robot_hw.h>

namespace mh5_hardware 
{


class MH5RobotHardware : public combined_robot_hw::CombinedRobotHW
{
public:
    virtual bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const {
        // Map from resource name to all controllers claiming it
        std::map<std::string, std::list<hardware_interface::ControllerInfo>> resource_map;

        // Populate a map of all controllers claiming individual resources.
        // We do this by iterating over every claimed resource of every hardware interface used by every controller
        for (const auto& controller_info : info) {
            for (const auto& claimed_resource : controller_info.claimed_resources) {
                for (const auto& iface_resource : claimed_resource.resources) {
                    std::string qual_name = claimed_resource.hardware_interface + '/' + iface_resource;
                    resource_map[qual_name].push_back(controller_info);
                }
            }
        }

        // Enforce resource exclusivity policy: No resource can be claimed by more than one controller
        bool in_conflict = false;
        for (const auto& resource_name_and_claiming_controllers : resource_map) {
            if (resource_name_and_claiming_controllers.second.size() > 1) {
                std::string controller_list;
                for (const auto& controller : resource_name_and_claiming_controllers.second)
                    controller_list += controller.name + ", ";
                ROS_WARN("Resource conflict on [%s].  Controllers = [%s]", resource_name_and_claiming_controllers.first.c_str(), controller_list.c_str());
                in_conflict = true;
            }
        }

        return in_conflict;
    }

};

} // namespace