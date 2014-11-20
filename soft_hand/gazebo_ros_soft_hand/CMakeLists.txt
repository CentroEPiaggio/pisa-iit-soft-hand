cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_soft_hand)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  control_toolbox
  controller_manager
  hardware_interface 
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    controller_manager
    pluginlib
    transmission_interface
  INCLUDE_DIRS include
  DEPENDS gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME} src/gazebo_ros_soft_hand_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(default_soft_hand_hw_sim src/default_soft_hand_hw_sim.cpp)
target_link_libraries(default_soft_hand_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(kinematic_ctrl_soft_hand_hw_sim src/kinematic_ctrl_soft_hand_hw_sim.cpp)
target_link_libraries(kinematic_ctrl_soft_hand_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Helper plugins
add_library(gazebo_ros_mimic_joint_plugin src/gazebo_ros_mimic_joint_plugin.cpp)
target_link_libraries(gazebo_ros_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

add_library(gazebo_ros_disable_link_plugin src/gazebo_ros_disable_link_plugin.cpp)
target_link_libraries(gazebo_ros_disable_link_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


## Install
install(TARGETS ${PROJECT_NAME} default_soft_hand_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(TARGETS ${PROJECT_NAME} kinematic_ctrl_soft_hand_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES soft_hand_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

