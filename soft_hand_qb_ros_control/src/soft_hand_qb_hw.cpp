/*
    soft_hand_qb_hw.cpp

    Purpose: A HW class for SoftHand. It makes use of the qb_interface topics: no low level operations are done.

    This is an improved version of the soft_hand_ros_control by Manuel Bonilla

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

#include "soft_hand_ros_control/soft_hand_qb_hw.h"

namespace soft_hand_qb_hw
{
  /* Default hardware interface constructor */
  SHHW::SHHW(ros::NodeHandle nh) : nh_(nh) {
    // Initializing subscribers and publishers
    hand_meas_sub = nh_.subscribe(std::string(HAND_MEAS_TOPIC), 1000, &soft_hand_qb_hw::SHHW::callBackMeas, this);
    hand_curr_sub = nh_.subscribe(std::string(HAND_CURR_TOPIC), 1000, &soft_hand_qb_hw::SHHW::callBackCurr, this);
    hand_ref_pub = nh_.advertise<qb_interface::handRef>(std::string(HAND_REF_TOPIC), 1000);

    // Initializing hand curr and meas variables
    hand_meas = 0.0; prev_hand_meas = 0.0;
    hand_curr = 0; prev_hand_curr = 0;
    prev_pos = 0.0;
  }

  /* Callback function for the position subscriber to qb_interface */
  void SHHW::callBackMeas(const qb_interface::handPosConstPtr& pos_msg){
    hand_meas = pos_msg->closure[0];
  }

  /* Callback function for the current subscriber to qb_interface */
  void SHHW::callBackCurr(const qb_interface::handCurrentConstPtr& curr_msg){
    hand_curr = curr_msg->current[0];
  }

  /* Function for initializing correctly the hand variables without NaNs */
  bool SHHW::initHandVars(){
    // Writing first non NaN measurement and current to prev variables
    bool hand_meas_good = false;
    bool hand_curr_good = false;

    while(!hand_meas_good && !hand_curr_good){
      if(!std::isnan(hand_meas)){
        prev_hand_meas = hand_meas;
        hand_meas_good = true;
        if(DEBUG) std::cout << "Found good hand measurement: " << prev_hand_meas << "." << std::endl;
      }
      if(!std::isnan(hand_curr)){
        prev_hand_curr = hand_curr;
        hand_curr_good = true;
        if(DEBUG) std::cout << "Found good hand current: " << prev_hand_curr << "." << std::endl;
      }

      if(DEBUG) std::cout << "Exiting initHandVars." << std::endl;
    }

  }

  /* Function for preliminary operations (run only once in the main before while) */
  bool SHHW::start(){

    // Construct and reset a new soft hand device structure (state and command storage)
    this->device_.reset(new SHHW::SHRDevice());

    // Get joint names from the parameter server (right_hand_names.yaml)
    if(ros::param::get("joints", this->device_->joint_names)){
      // Check if the number of synergies is the same as number of joints
      if(!(this->device_->joint_names.size()==N_SYN)){
        ROS_ERROR("This robot has 1 joint, you must specify 1 name only until more synergies are not included");
      } 
    } else {
      ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
      throw std::runtime_error("No joint name specification");
    }

    // Check if the robot description is loaded
    if(!(urdf_model_.initParam("robot_description"))){
      ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
      throw std::runtime_error("No URDF model available");
    }

    // Initialize and set to zero the state and command values (inline functions in h file)
    this->device_->init();
    this->device_->reset();

    // General joint pointer to store information
    boost::shared_ptr<const urdf::Joint> joint;

    // Create joint handles for each robot joint (1 for SoftHand)
    for(int i = 0; i < N_SYN; ++i){
      ROS_INFO_STREAM("Handling joint: " << this->device_->joint_names[i]);

      // Get the current joint (This is only for checking if the joint exists)
      joint = urdf_model_.getJoint(this->device_->joint_names[i]);
      if(!joint.get()){
        ROS_ERROR_STREAM("The specified joint "<< this->device_->joint_names[i] << " can't be found in the URDF model. Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
        throw std::runtime_error("Wrong joint name specification");
      }

      // Create a joint state handle and register it to the state interface
      hardware_interface::JointStateHandle state_handle(this->device_->joint_names[i],
          &this->device_->joint_position[i],
          &this->device_->joint_velocity[i],
          &this->device_->joint_effort[i]);

      state_interface_.registerHandle(state_handle);

      // Create a joint handle for the effort command and register it to the position interface
      hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(this->device_->joint_names[i]),
            &this->device_->joint_position_command[i]);

      position_interface_.registerHandle(joint_handle);

      /* Get the joint limits type (soft or hard), lower position limit, upper position limit, 
      and effort limit and registers the needed handles */
      registerJointLimits(this->device_->joint_names[i],
                          joint_handle,
                          &urdf_model_,
                          &this->device_->joint_lower_limits[i],
                          &this->device_->joint_upper_limits[i],
                          &this->device_->joint_effort_limits[i]);
    }

    // Register ros_control interfaces
    ROS_INFO("Registering state and position interfaces");
    this->registerInterface(&state_interface_);
    this->registerInterface(&position_interface_);
  }

  /* Function for reading from real robot (run in a while loop in the main) and updating the hand synergy joints */
  bool SHHW::read(ros::Time time, ros::Duration period){
    // Position and current are already read from hand by the subscribers but check if NaN
    float non_nan_hand_meas; short int non_nan_hand_curr;

    // Check for NaN in hand measurement and update prev meas if not Nan
    if(std::isnan(hand_meas)){
      non_nan_hand_meas = prev_hand_meas;
    } else {
      non_nan_hand_meas = hand_meas;
      prev_hand_meas = hand_meas;
    }

    // Check for NaN in hand current and update prev curr if not NaN
    if(std::isnan(hand_curr)){
      non_nan_hand_curr = prev_hand_curr;
    } else {
      non_nan_hand_curr = hand_curr;
      prev_hand_curr = hand_curr;
    }
      
    // Setting the inputs using the obtained hand_meas 
    static float inputs[2];                             // Two elements for future (SoftHand 2.5)
    inputs[0] = non_nan_hand_meas;
    inputs[1] = 0.0;

    // Setting the currents using the obtained hand_curr
    static short int currents[2];                       // Two elements for future (SoftHand 2.5)
    currents[0] = non_nan_hand_curr;
    currents[1] = 0;

    // Fill the state variables
    for(int j = 0; j < N_SYN; j++){
      this->device_->joint_position_prev[j] = this->device_->joint_position[j];
      this->device_->joint_position[j] = (double)(inputs[0]/MAX_HAND_MEAS);
      this->device_->joint_effort[j] = currents[0]*1.0;
      this->device_->joint_velocity[j] = filters::exponentialSmoothing((this->device_->joint_position[j]-this->device_->joint_position_prev[j])/period.toSec(), this->device_->joint_velocity[j], 0.2);
    }

    // std::cout << "Measurement is " << this->device_->joint_position[0] << "!" << std::endl;
    // std::cout << "Previous is " << this->device_->joint_position_prev[0] << "!" << std::endl;
    // std::cout << "Current is " << this->device_->joint_effort[0] << "!" << std::endl;

    return true;
  }

  /* Function for writing to real robot (run in a while loop in the main) the command from the controller manager */
  void SHHW::write(ros::Time time, ros::Duration period){
    // Enforce limits using the period
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);

    // Write to the hand the command given by controller manager
    float pos;
    pos = (float)(MAX_HAND_MEAS*this->device_->joint_position_command[0]);

    // Check for NaN in hand command and update prev pos if not Nan
    if(std::isnan(pos)){
      pos = prev_pos;
    } else {
      prev_pos = pos;
    }
    
    // std::cout << "Command is " << pos << "!" << std::endl;

    set_input(pos);

    return;
  }

  /* Function for writing the command to the real robot (using qb_interface), called in write() */
  void SHHW::set_input(float pos){
    static float inputs[2];

    inputs[0] = pos;
    inputs[1] = 0.0;

    // Creating the Hand Ref message for qb_interface
    qb_interface::handRef tmp_ref_msg;
    tmp_ref_msg.closure.push_back(inputs[0]);

    // Publishing the command to robot though qb_interface
    hand_ref_pub.publish(tmp_ref_msg);

    return;
  }

  /* Function for shutting down everything before exiting (run only once in the main after while) */
  void SHHW::stop(){
    // Just sleeping for giving others time to finish exiting
    usleep(2000000);
  }

  /* Function for eventually setting mode (NOT USED) */
  void SHHW::set_mode(){
    // the hand does not have something like this, anyway it is left here to keep the template
    return;
  }

  /*  Function for registering the limits of the joint specified by joint_name and joint_handle. 
      The limits are retrieved from the urdf_model.
      Gets the joint's type, lower position limit, upper position limit, and effort limit and
      registers the needed handles */
  void SHHW::registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const urdf::Model *const urdf_model,
                           double *const lower_limit, double *const upper_limit, 
                           double *const effort_limit){
    // Temporarily setting limits to infinity
    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();

    // Creating the limits interfaces
    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    // If the urdf is loaded, get the limits from it and set the interfaces
    if(urdf_model != NULL){
      const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
      if(urdf_joint != NULL){
        // Get limits from the URDF file.
        if(joint_limits_interface::getJointLimits(urdf_joint, limits)) has_limits = true;
        if(joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) has_soft_limits = true;
      }
    }

    // If no limits were found keep the infinities and exit
    if(!has_limits) return;

    // If there are position and effort limits change the infinities
    if(limits.has_position_limits){
      *lower_limit = limits.min_position;
      *upper_limit = limits.max_position;
    }

    if(limits.has_effort_limits) *effort_limit = limits.max_effort;

    // If there are also soft limits create appropriate handles and register them
    if(has_soft_limits){
      const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle, limits, soft_limits);
      ej_limits_interface_.registerHandle(limits_handle);
    } else {
      const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
      ej_sat_interface_.registerHandle(sat_handle);
    }
  }

}
