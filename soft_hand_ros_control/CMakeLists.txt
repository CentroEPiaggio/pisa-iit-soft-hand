cmake_minimum_required(VERSION 2.8.3)
project(soft_hand_ros_control)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

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
    pluginlib
    combined_robot_hw
)


## qbAPI is the default electronics the hand is sold with
## This assumes you cloned recursively to get he qbAPI sources
SET(HAND_TOOLS ../hand-tools/qbAPI)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES iit_hand_hw fake_hand_hw
    CATKIN_DEPENDS roscpp control_toolbox controller_interface controller_manager
                   hardware_interface realtime_tools tf urdf
                   joint_limits_interface pluginlib combined_robot_hw
#    DEPENDS
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${HAND_TOOLS}/src
)

## the qbAPI library
add_library(qbmove SHARED
    ${HAND_TOOLS}/src/qbmove_communications.cpp
)
    
add_library(iit_hand_hw src/iit_hand_hw.cpp)

add_library(fake_hand_hw src/fake_hand_hw.cpp)

add_executable(iit_hand_hw_node src/iit_hand_hw_node.cpp)

add_dependencies(iit_hand_hw ${catkin_EXPORTED_TARGETS}
                             qbmove
)

add_dependencies(fake_hand_hw ${catkin_EXPORTED_TARGETS})

add_dependencies(iit_hand_hw_node ${catkin_EXPORTED_TARGETS}
                                  iit_hand_hw
)

target_link_libraries(iit_hand_hw
    qbmove
    ${catkin_LIBRARIES}
)

target_link_libraries(fake_hand_hw
    ${catkin_LIBRARIES}
)

target_link_libraries(iit_hand_hw_node
    iit_hand_hw
    ${catkin_LIBRARIES}
)

install(TARGETS iit_hand_hw fake_hand_hw
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES iit_hand_hw_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
