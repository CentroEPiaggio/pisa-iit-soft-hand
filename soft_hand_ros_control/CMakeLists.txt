cmake_minimum_required(VERSION 2.8.3)
project(soft_hand_ros_control)

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
## This assumes you cloned recursively to get he qbAPI sources
SET(HAND_TOOLS ../hand-tools/qbAPI)

catkin_package()


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${HAND_TOOLS}/src
)

## the qbAPI library
add_library(qbmove SHARED
    ${HAND_TOOLS}/src/qbmovelibrary.cpp
    ${HAND_TOOLS}/src/qbmove_communications.cpp
)

# the ROS-driver
add_executable(soft_hand_hw
    src/soft_hand_hw.cpp
)
add_dependencies(soft_hand_hw
    qbmove
)
target_link_libraries(soft_hand_hw
    ${catkin_LIBRARIES}
    qbmove
)