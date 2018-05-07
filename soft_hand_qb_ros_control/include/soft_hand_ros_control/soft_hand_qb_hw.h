/*
    soft_hand_qb_hw.h

    Purpose: A HW class for SoftHand. It makes use of the qb_interface topics: no low level operations are done.

    This is an improved version of the soft_hand_ros_control by Manuel Bonilla

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Duration.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <control_toolbox/filters.h>
#include <urdf/model.h>

// QB tools
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include "soft_hand_ros_control/definitions.h"

// QB interface messages
#include <qb_interface/handPos.h>
#include <qb_interface/handCurrent.h>
#include <qb_interface/handRef.h>

namespace soft_hand_qb_hw {
	class SHHW : public hardware_interface::RobotHW {
		// PUBLIC VARIABLES AND FUNCTIONS
		public:
      /* Default hardware interface constructor */
    	SHHW(ros::NodeHandle nh);

      /* Callback functions for the subscribers to qb_interface */
      void callBackMeas(const qb_interface::handPosConstPtr& pos_msg);
      void callBackCurr(const qb_interface::handCurrentConstPtr& curr_msg);

      /* Function for initializing correctly the hand variables without NaNs */
      bool initHandVars();

    	/* Function for preliminary operations (run only once in the main before while) */
    	bool start();

      /*  Functions for reading and writing from/to real robot (run in a while loop in the main) 
          Passing also the current time for future usage.*/
    	bool read(ros::Time time, ros::Duration period);
    	void write(ros::Time time, ros::Duration period);

      /* Function for writing the command to the real robot (using qb_interface), called in write() */
      void set_input(float pos);

    	/* Function for shutting down everything before exiting (run only once in the main after while) */
    	void stop();

    	/* Function for eventually setting mode (NOT USED) */
    	void set_mode();

    	/* Function for eventually resetting (NOT USED) */
    	void reset();

    	/*  Function for registering the limits of the joint specified by joint_name and joint_handle. 
          The limits are retrieved from the urdf_model.
          Gets the joint's type, lower position limit, upper position limit, and effort limit and
          registers the needed handles */
    	void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const urdf::Model *const urdf_model,
                           double *const lower_limit, double *const upper_limit, 
                           double *const effort_limit);

    	// Structure for a SodtHand: data variables for joint state
    	struct SHRDevice {
    	// Names of the joints
      	std::vector<std::string> joint_names;

      	// Limits on the joints
     	std::vector<double> 
        joint_lower_limits,
        joint_upper_limits,
        joint_effort_limits;

      	// Joint state and command
      	std::vector<double>
        joint_position,
        joint_position_prev,
        joint_velocity,
        joint_effort,
        joint_position_command;

        // Function for initializing the above variables to the correct size
      	void init(){
        	joint_position.resize(N_SYN);
        	joint_position_prev.resize(N_SYN);
        	joint_velocity.resize(N_SYN);
        	joint_effort.resize(N_SYN);
        	joint_position_command.resize(N_SYN);
 
        	joint_lower_limits.resize(N_SYN);
        	joint_upper_limits.resize(N_SYN);
        	joint_effort_limits.resize(N_SYN);
        }

      	// Function for resetting the above variables to zeros
      	void reset(){
      		for (int j = 0; j < N_SYN; ++j) {
      			joint_position[j] = 0.0;
          		joint_position_prev[j] = 0.0;
          		joint_velocity[j] = 0.0;
          		joint_effort[j] = 0.0;
          		joint_position_command[j] = 0.0;
          	}
      	}
      };

      // Pointer to an above structure instance
    	boost::shared_ptr<SHHW::SHRDevice> device_;

    	// PRIVATE VARIABLES AND FUNCTIONS
    	private:
    	// Node handle
    	ros::NodeHandle nh_;

      // A subscriber for getting hand positions from qb_interface
      ros::Subscriber hand_meas_sub;

      // A subscriber for getting hand currents from qb_interface
      ros::Subscriber hand_curr_sub;

      // A publisher for publishing hand commands to qb_interface
      ros::Publisher hand_ref_pub;

      // Variables for temporarily storing hand measurement and current read from topics
      float hand_meas;
      short int hand_curr;

      // Variables for storing previous hand measurement and current which are not NaN (used for NaN problem)
      float prev_hand_meas;
      short int prev_hand_curr;

      // Variables for storing previous hand command which is not NaN (used for NaN problem)
      float prev_pos;

    	// Robot model
    	urdf::Model urdf_model_;

    	// Interfaces, QB tools only allow position interface, AFAIK
    	hardware_interface::JointStateInterface state_interface_;
    	hardware_interface::PositionJointInterface position_interface_;

    	joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
    	joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
    	joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;
    	joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface_;
    };
}