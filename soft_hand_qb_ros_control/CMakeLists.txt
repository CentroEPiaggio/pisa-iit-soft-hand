cmake_minimum_required(VERSION 2.8.3)
project(soft_hand_qb_ros_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    control_toolbox
    controller_interface
    controller_manager
    hardware_interface
    realtime_tools
    cmake_modules
    joint_limits_interface
    tf
    urdf
)


## qbAPI is the default electronics the hand is sold with
## This assumes you already have cloned and built qb_interface and qbAPI from NMMI
catkin_package()


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# the ROS-driver
add_executable(soft_hand_qb_hw
    src/sh_qb_hw_main.cpp
    src/soft_hand_qb_hw.cpp
)

target_link_libraries(soft_hand_qb_hw
    ${catkin_LIBRARIES}
)