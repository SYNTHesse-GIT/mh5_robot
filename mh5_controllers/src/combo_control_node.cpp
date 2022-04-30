#include <iostream>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <mh5_hardware/robot_hw.hpp>


int main(int argc, char** argv){
    ros::init(argc, argv, "combo_control_node");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    mh5_hardware::MH5RobotHardware hw;
    bool init_success = hw.init(nh,nh);

    controller_manager::ControllerManager cm(&hw,nh);

    ros::Rate rate(200); // 200Hz update rate

    ROS_INFO("combo_control started");
    while(ros::ok()){
        hw.read(ros::Time::now(), rate.expectedCycleTime());
        cm.update(ros::Time::now(), rate.expectedCycleTime());
        hw.write(ros::Time::now(), rate.expectedCycleTime());
        rate.sleep();
    }
    ROS_INFO("stopped combo_control");
    spinner.stop();
return 0;
}
