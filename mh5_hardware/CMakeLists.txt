cmake_minimum_required(VERSION 3.0.2)
project(mh5_hardware)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  combined_robot_hw
  joint_trajectory_controller
  forward_command_controller
  dynamixel_sdk
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager 
                 hardware_interface
                 combined_robot_hw
                 dynamixel_sdk
                 joint_trajectory_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mh5_hardware 
    src/dynamixel_bus.cpp
    src/dynamixel_device.cpp
    src/dynamixel_joint.cpp
    src/foot_sensor.cpp
    src/dynamixel_loop.cpp
# Removed temporary due to error on finding <i2c/smbus.h>
#    src/i2c_interface.cpp
#    src/LSM6DS3.cpp
)
target_link_libraries(mh5_hardware ${catkin_LIBRARIES})
#i2c)
