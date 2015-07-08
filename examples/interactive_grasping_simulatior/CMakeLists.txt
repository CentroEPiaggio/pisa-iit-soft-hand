cmake_minimum_required(VERSION 2.8.3)
project(interactive_grasping_simulator)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED 
	roscpp
	visualization_msgs
	interactive_markers
	sensor_msgs
	gazebo_msgs
	tf
)

catkin_package()

include_directories(include 
	${catkin_INCLUDE_DIRS}
)

add_executable(interactive_marker_server
	src/interactive_marker_server.cpp
)

target_link_libraries(interactive_marker_server
	${catkin_LIBRARIES}
)
