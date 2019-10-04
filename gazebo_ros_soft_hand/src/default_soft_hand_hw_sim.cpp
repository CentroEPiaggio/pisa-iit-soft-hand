/* 
   Author: Carlos Rosales, Hamal Marino
   Desc: Not any simulated robot, this is the soft hand hardware interface for simulation.
   Based on the Hardware Interface for any simulated robot in Gazebo by
   Dave Coleman, Johnathan Bohren
   Copyright (c) 2013, Open Source Robotics Foundation
   Copyright (c) 2013, The Johns Hopkins University
*/

#include <gazebo_ros_soft_hand/default_soft_hand_hw_sim.h>

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace gazebo_ros_soft_hand
{

bool DefaultSoftHandHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

  ROS_INFO("Default Soft Hand HW Sim Plugin initialization:");

  // IMPORTANT:
  // there should be only two transmissions, the hand must be loaded in robot_namespace/robot_description
  // for this to work, for now.
  // the one for the synergy joint, to be treated as in the default robot plugin
  transmission_interface::TransmissionInfo synergy_trans_info;
  // and the other one, whose control is not accessible from outside
  // instead it defines the state of the adaptive joints in 
  // the simulation through the adaptive transmission
  transmission_interface::TransmissionInfo adaptive_trans_info;

  // std::cout << "transmissions.size() " << transmissions.size() << std::endl;

  // select which of the transmission is which.
  if( transmissions[0].type_ == "transmission_interface/SimpleTransmission" )
  {
    synergy_trans_info = transmissions[0];
    adaptive_trans_info = transmissions[1];
  }
  else
  {
    synergy_trans_info = transmissions[1];
    adaptive_trans_info = transmissions[0];
  }

  /////////////////////////////////
  //  LOAD ADAPTIVE TRANSMISSION //
  /////////////////////////////////
  ROS_INFO("1. Load the adaptive transmission.");

  // this is specific, and uses a local library to load the adaptive transmission - instead of the more generic:
  TransmissionPluginLoader loader;
  boost::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader = loader.create(adaptive_trans_info.type_);
  
  assert(0 != transmission_loader);

  TransmissionSharedPtr transmission;
  const transmission_interface::TransmissionInfo& info = adaptive_trans_info;
  
  transmission = transmission_loader->load(info);
  
  assert(0 != transmission);

  adaptive_transmission_interface::AdaptiveSynergyTransmission* adaptive_trans = dynamic_cast<adaptive_transmission_interface::AdaptiveSynergyTransmission*>(transmission.get());

  adaptive_trans_ = new adaptive_transmission_interface::AdaptiveSynergyTransmission(adaptive_trans->getActuatorReduction(),
                                                                            adaptive_trans->getJointReduction(),
                                                                            adaptive_trans->getJointElastic(),
                                                                            adaptive_trans->getJointOffset());

  assert(0 != adaptive_trans_);


  // ubiquitous temporary gazebo joint and link
  gazebo::physics::JointPtr joint;
  gazebo::physics::LinkPtr link;

  ////////////////////////
  // INIT SYNERGY JOINT //
  ////////////////////////
  ROS_INFO("2. Init synergy joint.");

  std::vector<std::string> synergy_interfaces = synergy_trans_info.joints_[0].hardware_interfaces_;
  if (synergy_interfaces.empty())
  {
    ROS_WARN_STREAM_NAMED("default_soft_hand_hw_sim", "Joint " << synergy_trans_info.joints_[0].name_ <<
      " of transmission " << synergy_trans_info.name_ << " does not specify any hardware interface. " <<
      "Not adding it to the robot hardware simulation.");
  }
  else if (synergy_interfaces.size() > 1)
  {
    ROS_WARN_STREAM_NAMED("default_soft_hand_hw_sim", "Joint " << synergy_trans_info.joints_[0].name_ <<
      " of transmission " << synergy_trans_info.name_ << " specifies multiple hardware interfaces. " <<
      "Currently the default soft hand hardware simulation interface only supports one. Using the first entry!");
  }

  // Add data from transmission
  synergy_name_ = synergy_trans_info.joints_[0].name_;
  synergy_position_ = 1.0;
  synergy_velocity_ = 0.0;
  synergy_effort_ = 0.0;  // N/m for continuous joints
  synergy_effort_command_ = 0.0;
  synergy_position_command_ = 0.0;
  synergy_velocity_command_ = 0.0;

  const std::string& hardware_interface_syn = synergy_interfaces.front();

  // Debug
  ROS_DEBUG_STREAM_NAMED("default_soft_hand_hw_sim","Loading joint '" << synergy_name_
    << "' of type '" << hardware_interface_syn << "'");

  // Create joint state interface for the synergy joint
  js_interface_.registerHandle(hardware_interface::JointStateHandle(
    synergy_name_, &synergy_position_, &synergy_velocity_, &synergy_effort_));

  // The synergy joint only accepts PositionJointInterface, since this is how tipically the hand motor is controlled from ROS
  hardware_interface::JointHandle synergy_handle;
  if(hardware_interface_syn == "EffortJointInterface")
  {
    ROS_ERROR_STREAM("The synergy joint \"" << synergy_name_
      << " only accepts PositionJointInterface. Check the transmission defintion.");
    return false;
  }
  else if(hardware_interface_syn == "PositionJointInterface")
  {
    // Create position joint interface
    synergy_control_method_ = POSITION;
    synergy_handle = hardware_interface::JointHandle(js_interface_.getHandle(synergy_name_),
                                                   &synergy_position_command_);
    pj_interface_.registerHandle(synergy_handle);
  }
  else if(hardware_interface_syn == "VelocityJointInterface")
  {
    ROS_ERROR_STREAM("The synergy joint \"" << synergy_name_
      << " only accepts PositionJointInterface. Check the transmission defintion.");
    return false;
  }
  else
  {
    ROS_FATAL_STREAM_NAMED("default_soft_hand_hw_sim","No matching hardware interface found for '"
      << hardware_interface_syn );
    return false;
  }

  // Get the gazebo joint that corresponds to the robot joint.
  joint = parent_model->GetJoint(synergy_name_);
  if (!joint)
  {
    ROS_ERROR_STREAM("This robot has a joint named \"" << synergy_name_
      << "\" which is not in the gazebo model.");
    return false;
  }
  sim_synergy_ = joint;

  registerJointLimits(synergy_name_, synergy_handle, synergy_control_method_,
                      joint_limit_nh, urdf_model,
                      &synergy_lower_limit_, &synergy_upper_limit_,
                      &synergy_effort_limit_);

  if (synergy_control_method_ != EFFORT)
  {
    // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
    // joint->SetVelocity() to control the joint.
    const ros::NodeHandle nh(model_nh, robot_namespace + "/gazebo_ros_soft_hand/pid_gains/" +
                             synergy_name_);
    if (pid_controller_.init(nh, true))
    {
      switch (synergy_control_method_)
      {
        case POSITION:
          synergy_control_method_ = POSITION_PID;
          break;
      }
    }
    else
    {
      // joint->SetMaxForce() must be called if joint->SetAngle() or joint->SetVelocity() are
      // going to be called. joint->SetMaxForce() must *not* be called if joint->SetForce() is
      // going to be called.
      joint->SetParam("max_force", 0, synergy_effort_limit_);
    }
  }

  //////////////////////////
  // INIT ADAPTIVE JOINTS //
  //////////////////////////
  ROS_INFO("3. Init adaptive joints, link, and contact subscription.");

  // gains for pid controllers
  double kp = 10.0;
  double ki = 0.1;
  double kd = 0.5;
  double ki_max = 30.0;
  double ki_min = 0.01;

  // Resize vectors to our DOF
  n_dof_ = adaptive_trans_info.joints_.size();
  joint_names_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_contact_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);

  n_links_ = n_dof_; // +1 for the palm, don't forget
  link_names_.resize(n_links_);

  // Check that this transmission has one joint
  if(adaptive_trans_info.joints_.size() == 0)
  {
    ROS_WARN_STREAM_NAMED("default_soft_hand_hw_sim","Transmission " << adaptive_trans_info.name_
      << " has no associated joints.");
  }
  else if(adaptive_trans_info.joints_.size() > 19)
  {
    ROS_WARN_STREAM_NAMED("default_soft_hand_hw_sim","Transmission " << adaptive_trans_info.name_
      << " has more than nineteen joints. Currently the default soft hand hardware simulation "
      << " interface only supports nineteen.");
  }

  for(unsigned int j=0; j < n_dof_; ++j)
  {
    // Add data from transmissions
    joint_names_[j] = adaptive_trans_info.joints_[j].name_;
    joint_position_[j] = 0.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_contact_[j] = 0.0;
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    // take the first of the first
    const std::string& hardware_interface = adaptive_trans_info.joints_[0].hardware_interfaces_[0];

    // Debug
    ROS_DEBUG_STREAM_NAMED("default_soft_hand_hw_sim","Loading joint '" << joint_names_[j]
      << "' of type '" << hardware_interface << "'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    // Decide what kind of command interface the synergy joint has
    // and even when it is not controlled from outside, we register the joint limits to ensure
    // values are within the limits
    hardware_interface::JointHandle joint_handle;
    if(hardware_interface == "EffortJointInterface")
    {
      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);

      ej_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "PositionJointInterface")
    {
      // Create position joint interface, though it is not used for now
      joint_control_methods_[j] = POSITION;
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("default_soft_hand_hw_sim","No matching hardware interface found for '"
        << hardware_interface );
      return false;
    }

    // Get the gazebo joint that corresponds to the robot joint.
    //ROS_DEBUG_STREAM_NAMED("default_soft_hand_hw_sim", "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    //gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    joint = parent_model->GetJoint(joint_names_[j]);
    if (!joint)
    {
      ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
        << "\" which is not in the gazebo model.");
      return false;
    }
    joint->SetStiffness(0, adaptive_trans_->getJointElastic().at(j));
    sim_joints_.push_back(joint);

    // Get the gazebo links that corresponds to the robot links.
    // avoid the fake links
    if(j==0 || j==3 || j==7 || j==11 || j==15)
      link = joint->GetChild();
    else
      link = joint->GetChild()->GetChildJointsLinks().at(0);

    sim_links_.push_back(link);
    link_names_[j] = (link->GetName());
    // std::cout << link_names_[j] << std::endl;

    // Subscribe to contact topics
    sub_contacts_.push_back(model_nh.subscribe(model_nh.resolveName(std::string("/contacts/") + link->GetName()), 10, &DefaultSoftHandHWSim::getContacts, this));

    // create map for contact information
    link_applied_wrench_.insert( std::pair<std::string, KDL::Wrench>( std::string(link->GetName()), KDL::Wrench::Zero() ) );
    
    // Set up controls
    if (joint_control_methods_[j] != EFFORT)
    {
      // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
      // joint->SetVelocity() to control the joint.
      const ros::NodeHandle nh(model_nh, robot_namespace + "/gazebo_ros_soft_hand/pid_gains/" +
                               joint_names_[j]);

      pid_controllers_[j] = control_toolbox::Pid(kp, ki, kd, ki_max, ki_min);
      if(true)//pid_controllers_[j].init(nh, true))
      {
        switch (joint_control_methods_[j])
        {
          case POSITION:
            joint_control_methods_[j] = POSITION_PID;
            break;
        }
      }
      else
      {
        std::cout << "Couldn't init pid_controller " << j << " for joint " << joint_names_[j] <<
                     ", using estrict positioning in simulation" << std::endl;
        // joint->SetMaxForce() must be called if joint->SetAngle() or joint->SetVelocity() are
        // going to be called. joint->SetMaxForce() must *not* be called if joint->SetForce() is
        // going to be called.
        joint->SetParam("max_force", 0, joint_effort_limits_[j]);
      }
    }

  }

  //////////////////////////
  // INIT MIMIC JOINTS    //
  //////////////////////////
  ROS_INFO("4. Init mimic joints.");

  n_mimic_ = 14; // for now, a magic number, hopefully, when gazebo support mimic joints, this part will be useless
  joint_names_mimic_.resize(n_mimic_);
  joint_position_mimic_.resize(n_mimic_);
  joint_velocity_mimic_.resize(n_mimic_);
  joint_effort_mimic_.resize(n_mimic_);
  joint_effort_command_mimic_.resize(n_mimic_);
  joint_position_command_mimic_.resize(n_mimic_);
  joint_velocity_command_mimic_.resize(n_mimic_);

  // catch the names first
  int j_mimic = 0; // joint mimic counter
  for(unsigned int j=0; j < n_dof_; ++j)
  {
    if( j==1 || j==2 || 
        j==4 || j==5 || j==6 || 
        j==8 || j==9 || j==10 || 
        j==12 || j==13 || j==14 || 
        j==16 || j==17 || j==18 )
    { 
      joint_names_mimic_[j_mimic] = joint_names_[j] + std::string("_mimic");
      j_mimic++;
    }
  }
  
  for(unsigned int j=0; j < n_mimic_; ++j)
  {
    joint_position_mimic_[j] = 0.0;
    joint_velocity_mimic_[j] = 0.0;
    joint_effort_mimic_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_mimic_[j] = 0.0;
    joint_position_command_mimic_[j] = 0.0;
    joint_velocity_command_mimic_[j] = 0.0;

    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_mimic_[j], &joint_position_mimic_[j], &joint_velocity_mimic_[j], &joint_effort_mimic_[j]));

    // Get the gazebo joint that corresponds to the robot joint.
    //ROS_DEBUG_STREAM_NAMED("default_soft_hand_hw_sim", "Getting pointer to gazebo joint: "
    //  << joint_names_[j]);
    //gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    joint = parent_model->GetJoint(joint_names_mimic_[j]);
    if (!joint)
    {
      ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_mimic_[j]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_mimic_.push_back(joint);
  }

  ROS_INFO("5. Register joint interfaces.");

  // register joint  interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  ROS_INFO("6. Register transmission interfaces.");
  // transmission here is used a bit different from what expected

  // wrap simple transmission raw data - current state
  a_state_data_[0].position.push_back(&synergy_position_command_);
  a_state_data_[0].effort.push_back(&synergy_effort_);

  //a_state_data_[1].position.push_back(&synergy_position_);
  //a_state_data_[1].effort.push_back(&synergy_effort_);

  // this is not really commanding anything, it is needed for transmission calculations
  a_cmd_data_[0].position.push_back(&synergy_position_);
  //a_cmd_data_[0].effort.push_back(&synergy_effort_);


  a_cmd_data_[1].effort.push_back(&synergy_effort_);

  // wrap only the non-mimic joints
  for(int j = 0; j < n_dof_; ++j)
  {
    j_state_data_[0].position.push_back(&joint_position_[j]);
    j_state_data_[0].effort.push_back(&joint_effort_contact_[j]);

    j_cmd_data_[0].position.push_back(&joint_position_command_[j]);
    j_cmd_data_[0].effort.push_back(&joint_effort_contact_[j]);

    j_cmd_data_[1].position.push_back(&joint_position_[j]);
    j_cmd_data_[1].effort.push_back(&joint_effort_command_[j]);
  }

  // register transmissions to each interface, order not important here
//  jnt_to_act_pos_.registerHandle(transmission_interface::JointToActuatorPositionHandle(
//                                                            adaptive_trans_info.name_,
//                                                             adaptive_trans_,
//                                                             a_state_data_[0],
//                                                             j_state_data_[0]));

  jnt_to_act_eff_.registerHandle(transmission_interface::JointToActuatorEffortHandle(
                                                             adaptive_trans_info.name_,
                                                             adaptive_trans_,
                                                             a_state_data_[0],
                                                             j_state_data_[0]));

  act_to_jnt_pos_.registerHandle(transmission_interface::ActuatorToJointPositionHandle(
                                                             adaptive_trans_info.name_,
                                                             adaptive_trans_,
                                                             a_cmd_data_[0],
                                                             j_cmd_data_[0]));

  act_to_jnt_eff_.registerHandle(transmission_interface::ActuatorToJointEffortHandle(
                                                           adaptive_trans_info.name_,
                                                           adaptive_trans_,
                                                           a_cmd_data_[1],
                                                           j_cmd_data_[1]));

  ////////////////////////////////////////////
  /// INIT KINEMATIC TREES AND JOINT ARRAYS //
  ////////////////////////////////////////////
  ROS_INFO("7. Init kinematic tree, chains, solvers and joint arrays.");
  if (!kdl_parser::treeFromUrdfModel(*urdf_model, hand_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  ROS_INFO("Hand kinematic successfully parsed with %d joints, and %d segments.",hand_tree_.getNrOfJoints(),hand_tree_.getNrOfJoints());


  std::string root_name = robot_namespace + std::string("_palm_link"); //hand_tree_.getRootSegment()->first;

  // KINEMATICS
  // IMPORTANT:
  // KDL Calculates the jacobian expressed in the base frame of the chain, with reference point at the end effector of the *chain
  // The total wrench at links are expressed in the link frame, using the origin as the reference point
  // The resultant joint effort due to contact is reference frame independent, however, the MultiplyJacobian must be consistent in reference frames.

  // THUMB
  // get the chains
  hand_tree_.getChain(root_name, link_names_[0], thumb_to_knuckle_chain_);
  hand_tree_.getChain(root_name, link_names_[1], thumb_to_proximal_chain_);
  hand_tree_.getChain(root_name, link_names_[2], thumb_to_distal_chain_);

  // init the jacobian solvers
  thumb_to_knuckle_jac_solver_ = new KDL::ChainJntToJacSolver(thumb_to_knuckle_chain_);
  thumb_to_proximal_jac_solver_ = new KDL::ChainJntToJacSolver(thumb_to_proximal_chain_);
  thumb_to_distal_jac_solver_ = new KDL::ChainJntToJacSolver(thumb_to_distal_chain_);

  // init the forward kinematic solvers
  thumb_to_knuckle_fk_solver_ = new KDL::ChainFkSolverPos_recursive(thumb_to_knuckle_chain_);
  thumb_to_proximal_fk_solver_ = new KDL::ChainFkSolverPos_recursive(thumb_to_proximal_chain_);
  thumb_to_distal_fk_solver_ = new KDL::ChainFkSolverPos_recursive(thumb_to_distal_chain_);

  // resize the complete jntarrays
  thumb_to_knuckle_joints_ = KDL::JntArray(thumb_to_knuckle_chain_.getNrOfJoints());
  thumb_to_proximal_joints_ = KDL::JntArray(thumb_to_proximal_chain_.getNrOfJoints());
  thumb_to_distal_joints_ = KDL::JntArray(thumb_to_distal_chain_.getNrOfJoints());

  thumb_to_knuckle_jac_ = KDL::Jacobian(thumb_to_knuckle_chain_.getNrOfJoints());
  thumb_to_proximal_jac_ = KDL::Jacobian(thumb_to_proximal_chain_.getNrOfJoints());
  thumb_to_distal_jac_ = KDL::Jacobian(thumb_to_distal_chain_.getNrOfJoints());
  thumb_to_knuckle_jac_6x5_ = KDL::Jacobian(thumb_to_distal_chain_.getNrOfJoints());
  thumb_to_proximal_jac_6x5_ = KDL::Jacobian(thumb_to_distal_chain_.getNrOfJoints());
  SetToZero(thumb_to_knuckle_jac_6x5_);
  SetToZero(thumb_to_proximal_jac_6x5_);

  // FINGERS
  n_fingers_ = 4;
  fingers_to_knuckle_joints_.resize(n_fingers_);
  fingers_to_proximal_joints_.resize(n_fingers_);
  fingers_to_middle_joints_.resize(n_fingers_);
  fingers_to_distal_joints_.resize(n_fingers_);
  fingers_to_knuckle_chain_.resize(n_fingers_);
  fingers_to_proximal_chain_.resize(n_fingers_);
  fingers_to_middle_chain_.resize(n_fingers_);
  fingers_to_distal_chain_.resize(n_fingers_);
  fingers_to_knuckle_jac_solver_.resize(n_fingers_);
  fingers_to_proximal_jac_solver_.resize(n_fingers_);
  fingers_to_middle_jac_solver_.resize(n_fingers_);
  fingers_to_distal_jac_solver_.resize(n_fingers_);
  fingers_to_knuckle_fk_solver_.resize(n_fingers_);
  fingers_to_proximal_fk_solver_.resize(n_fingers_);
  fingers_to_middle_fk_solver_.resize(n_fingers_);
  fingers_to_distal_fk_solver_.resize(n_fingers_);
  fingers_to_knuckle_jac_.resize(n_fingers_);
  fingers_to_knuckle_jac_6x7_.resize(n_fingers_);
  fingers_to_proximal_jac_.resize(n_fingers_);
  fingers_to_proximal_jac_6x7_.resize(n_fingers_);
  fingers_to_middle_jac_.resize(n_fingers_);
  fingers_to_middle_jac_6x7_.resize(n_fingers_);
  fingers_to_distal_jac_.resize(n_fingers_);
  fingers_knuckle_frame_.resize(n_fingers_);
  fingers_proximal_frame_.resize(n_fingers_);
  fingers_middle_frame_.resize(n_fingers_);
  fingers_distal_frame_.resize(n_fingers_);

  for( int f = 0; f < n_fingers_ ; ++f)
  {
    hand_tree_.getChain(root_name, link_names_[3+n_fingers_], fingers_to_knuckle_chain_[f]);
    hand_tree_.getChain(root_name, link_names_[4+n_fingers_], fingers_to_proximal_chain_[f]);
    hand_tree_.getChain(root_name, link_names_[5+n_fingers_], fingers_to_middle_chain_[f]);
    hand_tree_.getChain(root_name, link_names_[6+n_fingers_], fingers_to_distal_chain_[f]);

    fingers_to_knuckle_jac_solver_[f] = new KDL::ChainJntToJacSolver(fingers_to_knuckle_chain_[f]);
    fingers_to_proximal_jac_solver_[f] = new KDL::ChainJntToJacSolver(fingers_to_proximal_chain_[f]);
    fingers_to_middle_jac_solver_[f] = new KDL::ChainJntToJacSolver(fingers_to_middle_chain_[f]);
    fingers_to_distal_jac_solver_[f] = new KDL::ChainJntToJacSolver(fingers_to_distal_chain_[f]);

    fingers_to_knuckle_fk_solver_[f] = new KDL::ChainFkSolverPos_recursive(fingers_to_knuckle_chain_[f]);
    fingers_to_proximal_fk_solver_[f] = new KDL::ChainFkSolverPos_recursive(fingers_to_proximal_chain_[f]);
    fingers_to_middle_fk_solver_[f] = new KDL::ChainFkSolverPos_recursive(fingers_to_middle_chain_[f]);
    fingers_to_distal_fk_solver_[f] = new KDL::ChainFkSolverPos_recursive(fingers_to_distal_chain_[f]);

    // resize the complete jntarrays
    fingers_to_knuckle_joints_[f] = KDL::JntArray(fingers_to_knuckle_chain_[f].getNrOfJoints());
    fingers_to_proximal_joints_[f] = KDL::JntArray(fingers_to_proximal_chain_[f].getNrOfJoints());
    fingers_to_middle_joints_[f] = KDL::JntArray(fingers_to_middle_chain_[f].getNrOfJoints());
    fingers_to_distal_joints_[f] = KDL::JntArray(fingers_to_distal_chain_[f].getNrOfJoints());

    fingers_to_knuckle_jac_[f] = KDL::Jacobian(fingers_to_knuckle_chain_[f].getNrOfJoints());
    fingers_to_proximal_jac_[f] = KDL::Jacobian(fingers_to_proximal_chain_[f].getNrOfJoints());
    fingers_to_middle_jac_[f] = KDL::Jacobian(fingers_to_middle_chain_[f].getNrOfJoints());
    fingers_to_distal_jac_[f] = KDL::Jacobian(fingers_to_distal_chain_[f].getNrOfJoints());
    fingers_to_knuckle_jac_6x7_[f] = KDL::Jacobian(fingers_to_distal_chain_[f].getNrOfJoints());
    fingers_to_proximal_jac_6x7_[f] = KDL::Jacobian(fingers_to_distal_chain_[f].getNrOfJoints());
    fingers_to_middle_jac_6x7_[f] = KDL::Jacobian(fingers_to_distal_chain_[f].getNrOfJoints());
    SetToZero(fingers_to_knuckle_jac_6x7_[f]);
    SetToZero(fingers_to_proximal_jac_6x7_[f]);
    SetToZero(fingers_to_middle_jac_6x7_[f]);
  }

   // just to test that everything goes well at loading time
   updateKinematics();
   updateStatics();

  ROS_INFO("Initizalization done!");

  return true;
}

void DefaultSoftHandHWSim::readSim(ros::Time time, ros::Duration period)
{
  // adaptive joints
  for(int j = 0; j < n_dof_; ++j)
  {
    joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], sim_joints_[j]->Position(0));
    joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }

  // mimic joints
  for(int j = 0; j < n_mimic_; ++j)
  {
    joint_position_mimic_[j] += angles::shortest_angular_distance(joint_position_mimic_[j], sim_joints_mimic_[j]->Position(0));
    joint_velocity_mimic_[j] = sim_joints_mimic_[j]->GetVelocity(0);
    joint_effort_mimic_[j] = sim_joints_mimic_[j]->GetForce((unsigned int)(0));
  }

  // read values from simulation (our hardware)
  synergy_position_ += angles::shortest_angular_distance(synergy_position_, sim_synergy_->Position(0));
  synergy_velocity_ = 0.0; //sim_synergy_->GetVelocity(0);
  // this is published below with the transmission propagate
  //synergy_effort_ = sim_synergy_->GetForce((unsigned int)(0));

  // obtain the actuator effort as a consequence of the synergy position and contacts
  jnt_to_act_eff_.propagate();

  // update joint arrays, jacobians, frames, change of frames, and column completion needed for the next function
  updateKinematics();
  // update the joint effort due to contact (it uses what it is in the contact message and updated values from the function above)
  updateStatics();
}

void DefaultSoftHandHWSim::writeSim(ros::Time time, ros::Duration period)
{
  // populate from synergy command to joint commands and control simulation
  // obtain the joint effort commands as a result of the synergy effort (computed using the contact forces) and the elastic at the joints
  // if no contact, the synergy effort is proportional to the synergy position
  // the resulting joint effort command is proportional to the difference between the synergy effort and the elastics
  act_to_jnt_pos_.propagate();
  act_to_jnt_eff_.propagate();

  // ensure limits
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);


  // synergy joint simulation control
  switch (synergy_control_method_)
  {
    case EFFORT:
      {
        const double effort = synergy_effort_command_;
        sim_synergy_->SetForce(0, effort);
      }
      break;

    case POSITION:
      sim_synergy_->SetPosition(0, synergy_position_command_);
      break;

    case POSITION_PID:
      {
        double error;
        angles::shortest_angular_distance_with_limits(synergy_position_,
                                                      synergy_position_command_,
                                                      synergy_lower_limit_,
                                                      synergy_upper_limit_,
                                                      error);

        const double effort_limit = synergy_effort_limit_;
        const double effort = clamp(pid_controller_.computeCommand(error, period),
                                    -effort_limit, effort_limit);
        sim_synergy_->SetForce(0, effort);
      }
      break;
  }

  // adaptive joints simulation control
  // joint mimic counter to mirror commands
  int j_mimic = 0;
  for(unsigned int j=0; j < n_dof_; ++j)
  {

    switch (joint_control_methods_[j])
    {
      case EFFORT:
        {
          const double effort = joint_effort_command_[j];

          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case POSITION:
        {           
          sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
        }
        break;

      case POSITION_PID:
        {
          // std::cout << "position command: " << joint_position_command_[j] << std::endl;
          double error;
          angles::shortest_angular_distance_with_limits(joint_position_[j],
                                                        joint_position_command_[j],
                                                        joint_lower_limits_[j],
                                                        joint_upper_limits_[j],
                                                        error);

          // this should be read from the urdf!
          const double effort_limit = 100.0;//joint_effort_limits_[j];

          const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                      -effort_limit, effort_limit);

          // std::cout << "effort: " << effort << std::endl;

          // remember that the first one is the synergy
          sim_joints_[j]->SetForce(0, effort);
        }
        break;
    }

    // mirror command values to mimic joints
      if( j==1 || j==2 || 
        j==4 || j==5 || j==6 || 
        j==8 || j==9 || j==10 || 
        j==12 || j==13 || j==14 || 
        j==16 || j==17 || j==18 )
    { 
      joint_position_command_mimic_[j_mimic] = joint_position_[j];
      j_mimic++;
    }
  }

  // mimic joints simulation control (only position mirrored from the adative joints)
  for(unsigned int j=0; j < n_mimic_; ++j)
  {
    sim_joints_mimic_[j]->SetPosition(0, joint_position_command_mimic_[j]);
  }
}

// Register the limits of the joint specified by joint_name and\ joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void DefaultSoftHandHWSim::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle,
                         const ControlMethod ctrl_method,
                         const ros::NodeHandle& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         double *const lower_limit, double *const upper_limit, 
                         double *const effort_limit)
{
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
          pj_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}

// uses the information published by the bumper plugins in the bodies
void DefaultSoftHandHWSim::getContacts(const gazebo_msgs::ContactsState &msg)
{
  // first
  std::string link_in_collision(msg.header.frame_id);

  // second
  KDL::Wrench wrench;

  if(msg.states.size() > 0)
  {
      // one plugin per link (for now), so index 0 is ok
      tf::wrenchMsgToKDL(msg.states.at(0).total_wrench, wrench);
      link_applied_wrench_.at( std::string(link_in_collision) ) = wrench;
  }
  else
  {
      wrench = KDL::Wrench::Zero();
      link_applied_wrench_.at( std::string(link_in_collision) ) = wrench;
  }
}

void DefaultSoftHandHWSim::updateKinematics()
{
  // global counters for adaptive and mimic joints
  int jp = 0;
  int jm = 0;

  // THUMB
  // update joint arrays
  thumb_to_knuckle_joints_(0) = joint_position_[jp];
  thumb_to_proximal_joints_(0) = joint_position_[jp];
  thumb_to_distal_joints_(0) = joint_position_[jp];
  jp++;
  thumb_to_proximal_joints_(1) = joint_position_[jp];
  thumb_to_distal_joints_(1) = joint_position_[jp];
  jp++;
  thumb_to_proximal_joints_(2) = joint_position_mimic_[jm];
  thumb_to_distal_joints_(2) = joint_position_mimic_[jm];
  jm++;
  thumb_to_distal_joints_(3) = joint_position_[jp];
  jp++;
  thumb_to_distal_joints_(4) = joint_position_mimic_[jm];
  jm++;

  // update jacobians
  thumb_to_knuckle_jac_solver_->JntToJac(thumb_to_knuckle_joints_, thumb_to_knuckle_jac_);
  thumb_to_proximal_jac_solver_->JntToJac(thumb_to_proximal_joints_, thumb_to_proximal_jac_);
  thumb_to_distal_jac_solver_->JntToJac(thumb_to_distal_joints_, thumb_to_distal_jac_);

  // update frames
  thumb_to_knuckle_fk_solver_->JntToCart(thumb_to_knuckle_joints_, thumb_knuckle_frame_);
  thumb_to_proximal_fk_solver_->JntToCart(thumb_to_proximal_joints_, thumb_proximal_frame_);
  thumb_to_distal_fk_solver_->JntToCart(thumb_to_distal_joints_, thumb_distal_frame_);

  // update jacobian with chage of frame
  thumb_to_knuckle_jac_.changeRefFrame(thumb_knuckle_frame_);
  thumb_to_proximal_jac_.changeRefFrame(thumb_proximal_frame_);
  thumb_to_distal_jac_.changeRefFrame(thumb_distal_frame_);

  // update jacobian with column completion to facilitate effort computation
  for(int i=0; i<thumb_to_knuckle_jac_.columns(); ++i)
  {
    thumb_to_knuckle_jac_6x5_.setColumn(i, thumb_to_knuckle_jac_.getColumn(i));
  }
  for(int i=0; i<thumb_to_proximal_jac_.columns(); ++i)
  {
    thumb_to_proximal_jac_6x5_.setColumn(i, thumb_to_proximal_jac_.getColumn(i));
  }

  // // print to test
  // std::cout << "thumb_to_knuckle_jac_6x5_: " << std::endl << thumb_to_knuckle_jac_6x5_.data << std::endl;
  // std::cout << "thumb_to_proximal_jac_6x5_: " << std::endl << thumb_to_proximal_jac_6x5_.data << std::endl;
  // std::cout << "thumb_to_distal_jac_: " << std::endl << thumb_to_distal_jac_.data << std::endl;

  // FINGERS
  for(int f = 0; f < n_fingers_; ++f)
  {
    // update joint arrays
    fingers_to_knuckle_joints_.at(f)(0) = joint_position_[jp];
    fingers_to_proximal_joints_.at(f)(0) = joint_position_[jp];
    fingers_to_middle_joints_.at(f)(0) = joint_position_[jp];
    fingers_to_distal_joints_.at(f)(0) = joint_position_[jp];
    jp++;
    fingers_to_proximal_joints_.at(f)(1) = joint_position_[jp];
    fingers_to_middle_joints_.at(f)(1) = joint_position_[jp];
    fingers_to_distal_joints_.at(f)(1) = joint_position_[jp];
    jp++;
    fingers_to_proximal_joints_.at(f)(2) = joint_position_mimic_[jm];
    fingers_to_middle_joints_.at(f)(2) = joint_position_mimic_[jm];
    fingers_to_distal_joints_.at(f)(2) = joint_position_mimic_[jm];
    jm++;
    fingers_to_middle_joints_.at(f)(3) = joint_position_[jp];
    fingers_to_distal_joints_.at(f)(3) = joint_position_[jp];
    jp++;
    fingers_to_middle_joints_.at(f)(4) = joint_position_mimic_[jm];
    fingers_to_distal_joints_.at(f)(4) = joint_position_mimic_[jm];
    jm++;
    fingers_to_distal_joints_.at(f)(5) = joint_position_[jp];
    jp++;
    fingers_to_distal_joints_.at(f)(6) = joint_position_mimic_[jm];
    jm++;

    // update jacobians
    fingers_to_knuckle_jac_solver_[f]->JntToJac(fingers_to_knuckle_joints_[f], fingers_to_knuckle_jac_[f]);
    fingers_to_proximal_jac_solver_[f]->JntToJac(fingers_to_proximal_joints_[f], fingers_to_proximal_jac_[f]);
    fingers_to_middle_jac_solver_[f]->JntToJac(fingers_to_middle_joints_[f], fingers_to_middle_jac_[f]);
    fingers_to_distal_jac_solver_[f]->JntToJac(fingers_to_distal_joints_[f], fingers_to_distal_jac_[f]);

    // update frames
    fingers_to_knuckle_fk_solver_[f]->JntToCart(fingers_to_knuckle_joints_[f], fingers_knuckle_frame_[f]);
    fingers_to_proximal_fk_solver_[f]->JntToCart(fingers_to_proximal_joints_[f], fingers_proximal_frame_[f]);
    fingers_to_middle_fk_solver_[f]->JntToCart(fingers_to_middle_joints_[f], fingers_middle_frame_[f]);
    fingers_to_distal_fk_solver_[f]->JntToCart(fingers_to_distal_joints_[f], fingers_distal_frame_[f]);

    // update jacobian with chage of frame
    fingers_to_knuckle_jac_[f].changeRefFrame(fingers_knuckle_frame_[f]);
    fingers_to_proximal_jac_[f].changeRefFrame(fingers_proximal_frame_[f]);
    fingers_to_middle_jac_[f].changeRefFrame(fingers_middle_frame_[f]);
    fingers_to_distal_jac_[f].changeRefFrame(fingers_distal_frame_[f]);

    // update jacobian with column completion to facilitate effort computation
    for(int i=0; i<fingers_to_knuckle_jac_[f].columns(); ++i)
    {
      fingers_to_knuckle_jac_6x7_[f].setColumn(i, fingers_to_knuckle_jac_[f].getColumn(i));
    }
    for(int i=0; i<fingers_to_proximal_jac_[f].columns(); ++i)
    {
      fingers_to_proximal_jac_6x7_[f].setColumn(i, fingers_to_proximal_jac_[f].getColumn(i));
    }
    for(int i=0; i<fingers_to_middle_jac_[f].columns(); ++i)
    {
      fingers_to_middle_jac_6x7_[f].setColumn(i, fingers_to_middle_jac_[f].getColumn(i));
    }

    // // print to test
    // std::cout << "fingers_to_knuckle_jac_6x7_ for finger: " << f << std::endl << fingers_to_knuckle_jac_6x7_[f].data << std::endl;
    // std::cout << "fingers_to_proximal_jac_6x7_ for finger: " << f << std::endl << fingers_to_proximal_jac_6x7_[f].data << std::endl;
    // std::cout << "fingers_to_middle_jac_6x7_ for finger: " << f << std::endl << fingers_to_middle_jac_6x7_[f].data << std::endl;
    // std::cout << "fingers_to_distal_jac_ for finger: " << f << std::endl << fingers_to_distal_jac_[f].data << std::endl;
  }
}

// sum all contributions of each wrench applied to links per each joint
void DefaultSoftHandHWSim::updateStatics()
{
  // force scale due to high contact force that generate too high efforts
  // ToDo: scale everything well to avoid this magic value
  float effort_scale = -1*0.001;

  // link counter
  unsigned int lk = 0;

  // temporary wrench and transpose jacobian
  KDL::Wrench wrench;
  KDL::JntArray effort(5);
  KDL::JntArray total_thumb_effort(5);
  SetToZero(total_thumb_effort);

  // THUMB
  // knuckle
  wrench = link_applied_wrench_.at(sim_links_.at(lk)->GetName());
  MultiplyJacobian(thumb_to_knuckle_jac_6x5_, wrench, effort);
  KDL::Add(total_thumb_effort, effort, total_thumb_effort);
  lk++;

  // proximal
  wrench = link_applied_wrench_.at(sim_links_.at(lk)->GetName());
  MultiplyJacobian(thumb_to_proximal_jac_6x5_, wrench, effort);
  KDL::Add(total_thumb_effort, effort, total_thumb_effort);
  lk++;

  // distal
  wrench = link_applied_wrench_.at(sim_links_.at(lk)->GetName());
  MultiplyJacobian(thumb_to_distal_jac_, wrench, effort);
  KDL::Add(total_thumb_effort, effort, total_thumb_effort);
  lk++;

  joint_effort_contact_[0] = total_thumb_effort(0)*effort_scale;
  joint_effort_contact_[1] = total_thumb_effort(1)*effort_scale;
  joint_effort_contact_[2] = total_thumb_effort(3)*effort_scale;

  KDL::Wrench wrench_finger;
  KDL::JntArray effort_finger(7);
  KDL::JntArray total_finger_effort(7);

  // FINGERS
  for(int f =0; f < n_fingers_; ++f)
  {
    SetToZero(total_finger_effort);

    // knuckle
    wrench_finger = link_applied_wrench_.at(sim_links_.at(lk)->GetName());
    MultiplyJacobian(fingers_to_knuckle_jac_6x7_[f], wrench_finger, effort_finger);
    KDL::Add(total_finger_effort, effort_finger, total_finger_effort);
    lk++;

    // proximal
    wrench_finger = link_applied_wrench_.at(sim_links_.at(lk)->GetName());
    MultiplyJacobian(fingers_to_proximal_jac_6x7_[f], wrench_finger, effort_finger);
    KDL::Add(total_finger_effort, effort_finger, total_finger_effort);
    lk++;

    // middle
    wrench_finger = link_applied_wrench_.at(sim_links_.at(lk)->GetName());
    MultiplyJacobian(fingers_to_middle_jac_6x7_[f], wrench_finger, effort_finger);
    KDL::Add(total_finger_effort, effort_finger, total_finger_effort);
    lk++;

    // distal
    wrench_finger = link_applied_wrench_.at(sim_links_.at(lk)->GetName());
    MultiplyJacobian(fingers_to_distal_jac_[f], wrench_finger, effort_finger);
    KDL::Add(total_finger_effort, effort_finger, total_finger_effort);
    lk++;

    uint nr_joints_per_finger(4);
    joint_effort_contact_[3 + f*nr_joints_per_finger] = total_finger_effort(0)*effort_scale;
    joint_effort_contact_[4 + f*nr_joints_per_finger] = total_finger_effort(1)*effort_scale;
    joint_effort_contact_[5 + f*nr_joints_per_finger] = total_finger_effort(3)*effort_scale;
    joint_effort_contact_[6 + f*nr_joints_per_finger] = total_finger_effort(5)*effort_scale;
  }
}

// taken from http://www.orocos.org/forum/rtt/rtt-dev/jacobian-wrench-joint-torques
// however the tips didn't work for me
void DefaultSoftHandHWSim::MultiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest)
{
  Eigen::Matrix<double,6,1> w;
  w(0) = src.force(0);
  w(1) = src.force(1);
  w(2) = src.force(2);
  w(3) = src.torque(0);
  w(4) = src.torque(1);
  w(5) = src.torque(2);

  Eigen::MatrixXd j(jac.rows(), jac.columns());
  j = jac.data;
  j.transposeInPlace();

  Eigen::VectorXd t(jac.columns());
  t = j*w;

  dest.resize(jac.columns());
  for (unsigned i=0; i<jac.columns(); i++)
    dest(i) = t(i);
}

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_soft_hand::DefaultSoftHandHWSim, gazebo_ros_soft_hand::SoftHandHWSim)
