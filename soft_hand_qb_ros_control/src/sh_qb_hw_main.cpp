/*
    sh_qb_hw_main.cpp

    Purpose: a ROS node which launches the hardware_interface for SoftHand. 
    It makes use of the qb_interface topics: no low level operations are done.

    This is an improved version of the soft_hand_ros_control by Manuel Bonilla

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

#include "soft_hand_ros_control/soft_hand_qb_hw.h"

// GLOBAL VARIABLES
bool g_quit = false;

//--------------------------------------------------------------------------------------------------//
// AUXILIARY FUNCTIONS
//--------------------------------------------------------------------------------------------------//

void quitRequested(int sig){
  g_quit = true;
}

//--------------------------------------------------------------------------------------------------//
// MAIN
//--------------------------------------------------------------------------------------------------//

int main( int argc, char** argv ){
  // Initialize ROS node
  ros::init(argc, argv, "soft_hand_qb_hw_interface", ros::init_options::NoSigintHandler);

  // Use an Asyncronous ROS spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Use the custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Construct a SoftHand HW interface class element
  ros::NodeHandle sh_nh;
  soft_hand_qb_hw::SHHW sh_robot(sh_nh);

  // Solve the problem of first NaN readings for prev_hand_meas and prev_hand_curr
  sh_robot.initHandVars();

  // Initialize and set the necessary variables inside the SoftHand HW interface
  sh_robot.start();

  // Timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);

  /*  Period used in read (to estimate joint velocities from current and previous positions)
      and in write (to enforce limits of the robot) */
  ros::Duration period(1.0);

  // Controller manager
  controller_manager::ControllerManager manager(&sh_robot, sh_nh);

  // Run as fast as possible
  while(!g_quit){
    // Get the time and set the period (clock_gettime returns 0 for success)
    if(!clock_gettime(CLOCK_REALTIME, &ts)){
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } else {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    } 

    // Read the state from the soft hand (if not possible, prepare to quit)
    if(!sh_robot.read(now, period)){
      g_quit = true;
      break;
    }

    /*  Update the controller manager with the read position, velocity and effort and write the command
        to the robot class */
    manager.update(now, period);

    // Write the robot class command (given by the controller manager) to the SoftHand robot
    sh_robot.write(now, period);
  }

  std::cerr <<" Stopping SoftHand AsyncSpinner..." << std::endl;
  spinner.stop();

  std::cerr << "Stopping SoftHand..." << std::endl;
  sh_robot.stop();

  std::cerr << "Done!" << std::endl;

  return 0;
}