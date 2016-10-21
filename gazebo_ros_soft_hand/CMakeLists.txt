cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_soft_hand)

add_definitions (-fpermissive -std=c++11)

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
  kdl_conversions
  kdl_parser
  geometry_msgs
  cmake_modules
)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

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

## Libraries
add_library(adaptive_transmission_loader_plugin
  src/adaptive_synergy_transmission_loader.cpp
)
add_dependencies(adaptive_transmission_loader_plugin transmission_interface_loader)
target_link_libraries(adaptive_transmission_loader_plugin ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/gazebo_ros_soft_hand_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(default_soft_hand_hw_sim src/default_soft_hand_hw_sim.cpp)
add_dependencies(default_soft_hand_hw_sim
    adaptive_transmission_loader_plugin
)
target_link_libraries(default_soft_hand_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
    adaptive_transmission_loader_plugin
)

add_library(kinematic_ctrl_soft_hand_hw_sim src/kinematic_ctrl_soft_hand_hw_sim.cpp)
add_dependencies(kinematic_ctrl_soft_hand_hw_sim
    adaptive_transmission_loader_plugin
)
target_link_libraries(kinematic_ctrl_soft_hand_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
    adaptive_transmission_loader_plugin
)

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

install(FILES adaptive_transmission_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(adaptive_synergy_transmission_test test/adaptive_synergy_transmission_test.cpp)

  catkin_add_gtest(adaptive_synergy_transmission_loader_test test/adaptive_synergy_transmission_loader_test.cpp)
  target_link_libraries(adaptive_synergy_transmission_loader_test
    ${catkin_LIBRARIES}
    adaptive_transmission_loader_plugin
  )
endif()
