cmake_minimum_required(VERSION 2.8.3)
project(grasp_state_publisher)

find_package(catkin REQUIRED 
	roscpp
	message_generation
	std_msgs
	sensor_msgs
	message_filters
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
	FILES
	GraspState.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grasp_state_publisher
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(soft_grasp_observer
	src/soft_grasp_observer.cpp
)

add_executable(grasp_state_publisher 
	src/grasp_state_publisher.cpp)

add_dependencies(grasp_state_publisher 
	grasp_state_publisher_generate_messages_cpp
	soft_grasp_observer
)

## Specify libraries to link a library or executable target against
target_link_libraries(grasp_state_publisher
	soft_grasp_observer
	orocos-bfl
	${catkin_LIBRARIES}
)
