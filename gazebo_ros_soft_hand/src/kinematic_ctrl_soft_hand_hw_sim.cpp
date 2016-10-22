/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  Copyright (c) 2014, Research Center "E. Piaggio"
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Johnathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo

   Author: Carlos Rosales
   Desc: Not any simulated robot, this is the soft hand hardware interface for simulation
*/


#include <gazebo_ros_soft_hand/kinematic_ctrl_soft_hand_hw_sim.h>

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace gazebo_ros_soft_hand
{


bool KinematicCtrlSoftHandHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

  ROS_INFO("Kinematic Control Soft Hand HW Sim Plugin initialization:");

  // there should be only two transmissions
  // the one for the synergy joint, to be treated as in the kinematic_ctrl robot plugin
  transmission_interface::TransmissionInfo synergy_trans_info;
  // and the other one, whose control is not accessible from outside
  // instead it defines the state of the adaptive joints in 
  // the simulation (in fact it requires values from the simulation)
  // which are controlled
  transmission_interface::TransmissionInfo adaptive_trans_info;

  // select which of the transmission is which. we expect they were filled correctly
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
  //  INIT ADAPTIVE TRANSMISSION //
  /////////////////////////////////
  TransmissionPluginLoader loader;
  boost::shared_ptr<transmission_interface::TransmissionLoader> transmission_loader = loader.create(adaptive_trans_info.type_);

  assert(0 != transmission_loader);

  TransmissionPtr transmission;
  const transmission_interface::TransmissionInfo& info = adaptive_trans_info;
  
  transmission = transmission_loader->load(info);
  
  assert(0 != transmission);

  // Validate transmission
  adaptive_trans_ = dynamic_cast<adaptive_transmission_interface::AdaptiveSynergyTransmission*>(transmission.get());

  assert(0 != adaptive_trans_);

  // Resize vectors to our DOF + our mimic joints
  n_dof_ = adaptive_trans_info.joints_.size() + 14;
  joint_names_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  // temporary gazebo joint
  gazebo::physics::JointPtr joint;

  ////////////////////////
  // INIT SYNERGY JOINT //
  ////////////////////////

  ROS_INFO("1. Initializing the synergy joint.");

  std::vector<std::string> synergy_interfaces = synergy_trans_info.joints_[0].hardware_interfaces_;
  if (synergy_interfaces.empty())
  {
    ROS_WARN_STREAM_NAMED("kinematic_ctrl_soft_hand_hw_sim", "Joint " << synergy_trans_info.joints_[0].name_ <<
      " of transmission " << synergy_trans_info.name_ << " does not specify any hardware interface. " <<
      "Not adding it to the robot hardware simulation.");
  }
  else if (synergy_interfaces.size() > 1)
  {
    ROS_WARN_STREAM_NAMED("kinematic_ctrl_soft_hand_hw_sim", "Joint " << synergy_trans_info.joints_[0].name_ <<
      " of transmission " << synergy_trans_info.name_ << " specifies multiple hardware interfaces. " <<
      "Currently the kinematic_ctrl soft hand hardware simulation interface only supports one. Using the first entry!");
  }

  // Add data from transmission
  synergy_name_ = synergy_trans_info.joints_[0].name_;
  synergy_position_ = 1.0;
  synergy_velocity_ = 0.0;
  synergy_effort_ = 1.0;  // N/m for continuous joints
  synergy_effort_command_ = 0.0;
  synergy_position_command_ = 0.0;
  synergy_velocity_command_ = 0.0;

  const std::string& hardware_interface_syn = synergy_interfaces.front();

  // Debug
  ROS_DEBUG_STREAM_NAMED("kinematic_ctrl_soft_hand_hw_sim","Loading joint '" << synergy_name_
    << "' of type '" << hardware_interface_syn << "'");

  // Create joint state interface for the synergy joint
  js_interface_.registerHandle(hardware_interface::JointStateHandle(
    synergy_name_, &synergy_position_, &synergy_velocity_, &synergy_effort_));

  // Decide what kind of command interface the synergy joint has
  hardware_interface::JointHandle synergy_handle;
  if(hardware_interface_syn == "EffortJointInterface")
  {
    // Create effort joint interface
    synergy_control_method_ = EFFORT;
    synergy_handle = hardware_interface::JointHandle(js_interface_.getHandle(synergy_name_),
                                                   &synergy_effort_command_);

    ej_interface_.registerHandle(synergy_handle);
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
    // Create velocity joint interface
    synergy_control_method_ = VELOCITY;
    synergy_handle = hardware_interface::JointHandle(js_interface_.getHandle(synergy_name_),
                                                   &synergy_velocity_command_);
    vj_interface_.registerHandle(synergy_handle);
  }
  else
  {
    ROS_FATAL_STREAM_NAMED("kinematic_ctrl_soft_hand_hw_sim","No matching hardware interface found for '"
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
        case VELOCITY:
          synergy_control_method_ = VELOCITY_PID;
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

  ROS_INFO("2. Initializing adaptive and mimic joints.");

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

  int j_mimic = 19; // joint mimic counter
  for(unsigned int j=0; j < n_dof_; ++j)
  {
    // Add data from transmission
    if(j<19)
    {
      joint_names_[j] = adaptive_trans_info.joints_[j].name_;  
    }

    if( j==1 || j==2 || 
        j==4 || j==5 || j==6 || 
        j==8 || j==9 || j==10 || 
        j==12 || j==13 || j==14 || 
        j==16 || j==17 || j==18 )
    { 
      joint_names_[j_mimic] = adaptive_trans_info.joints_[j].name_ + std::string("_mimic");
      j_mimic++;
    }
    
    // std::cout << joint_names_[j] << std::endl;

    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    // take the first of the first
    const std::string& hardware_interface = adaptive_trans_info.joints_[0].hardware_interfaces_[0];

    // Debug
    ROS_DEBUG_STREAM_NAMED("default_soft_hand_hw_sim","Loading joint '" << joint_names_[j]
      << "' of type '" << hardware_interface << "'");

    // Create joint state interface for all joints
    /*js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));*/

    // Decide what kind of command interface the synergy joint has
    hardware_interface::JointHandle joint_handle;
    if(hardware_interface == "EffortJointInterface")
    {
      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
      /*joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);
      ej_interface_.registerHandle(joint_handle);*/
    }
    else if(hardware_interface == "PositionJointInterface")
    {
      // Create position joint interface
      joint_control_methods_[j] = POSITION;
      /*joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle);*/
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
    sim_joints_.push_back(joint);

    /*registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
                        joint_limit_nh, urdf_model,
                        &joint_lower_limits_[j], &joint_upper_limits_[j],
                        &joint_effort_limits_[j]);*/
    
    if (joint_control_methods_[j] != EFFORT)
    {
      // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
      // joint->SetVelocity() to control the joint.
      const ros::NodeHandle nh(model_nh, robot_namespace + "/gazebo_ros_soft_hand/pid_gains/" +
                               joint_names_[j]);

      pid_controllers_[j] = control_toolbox::Pid(10.0,0.5,0.1,30.0,0.01);
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

  ROS_INFO("3. Registering joint interfaces.");

  // register joint  interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  ROS_INFO("Initizalization done!");

  return true;
}

void KinematicCtrlSoftHandHWSim::readSim(ros::Time time, ros::Duration period)
{
  // read values from simulation (our hardware)
  synergy_position_ += angles::shortest_angular_distance(synergy_position_,
                          sim_synergy_->GetAngle(0).Radian());
  synergy_velocity_ = sim_synergy_->GetVelocity(0);
  synergy_effort_ = sim_synergy_->GetForce((unsigned int)(0));
}

void KinematicCtrlSoftHandHWSim::writeSim(ros::Time time, ros::Duration period)
{

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

  int jmimic = 19;
  // adaptive joints simulation control (position PID)
  for(unsigned int j=0; j < n_dof_; ++j)
  {
    switch (joint_control_methods_[j])
    {
      case EFFORT:
        {
          // mirror command values to the mimic ones
          if( j==1 || j==2 || 
              j==4 || j==5 || j==6 || 
              j==8 || j==9 || j==10 || 
              j==12 || j==13 || j==14 || 
              j==16 || j==17 || j==18 )
          {
            joint_effort_command_[jmimic] = joint_effort_command_[j];
          }
          jmimic++;

          // std::cout << "joint_effort_command_ " << j << " " << joint_effort_command_[j] << std::endl;

          const double effort = joint_effort_command_[j];

          // remember that the first one is the synergy
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case POSITION:
        {
           // mirror command values to the mimic ones
           if( j==1 || j==2 ||
               j==4 || j==5 || j==6 ||
               j==8 || j==9 || j==10 ||
               j==12 || j==13 || j==14 ||
               j==16 || j==17 || j==18 )
           {
             joint_position_command_[jmimic] = joint_position_command_[j];
             jmimic++;
           }
           

          // std::cout << "joint_position_command_ " << j << " " << joint_position_command_[j] << std::endl;

          // remember that the first one is the synergy
          sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
        }
        break;

      case POSITION_PID:
        {
          // mirror command values to the mimic ones
          if( j==1 || j==2 || 
              j==4 || j==5 || j==6 || 
              j==8 || j==9 || j==10 || 
              j==12 || j==13 || j==14 || 
              j==16 || j==17 || j==18 )
          {
            joint_position_command_[jmimic] = joint_position_command_[j];
            jmimic++;
          }
          

          // std::cout << "joint_position_ for " << joint_names_[j] << " " << joint_position_[j] << std::endl;
          // std::cout << "joint_position_command_ for " << joint_names_[j] << " " << joint_position_command_[j] << std::endl;

          double error;
          angles::shortest_angular_distance_with_limits(joint_position_[j],
                                                        joint_position_command_[j],
                                                        joint_lower_limits_[j],
                                                        joint_upper_limits_[j],
                                                        error);

          // std::cout << "error for joint " << joint_names_[j] << " " << error << std::endl;

          // this should be read from the urdf!
          const double effort_limit = 100;//joint_effort_limits_[j];

          // std::cout << "effort limit for joint " << joint_names_[j] << " " << effort_limit << std::endl;

          const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                      -effort_limit, effort_limit);

          // std::cout << "set effort for joint " << joint_names_[j] << " " << effort << std::endl;
          // remember that the first one is the synergy
          sim_joints_[j]->SetForce(0, effort);
        }
        break;
    }
  }
}

// Register the limits of the joint specified by joint_name and\ joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void KinematicCtrlSoftHandHWSim::registerJointLimits(const std::string& joint_name,
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
    const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
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

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_soft_hand::KinematicCtrlSoftHandHWSim, gazebo_ros_soft_hand::SoftHandHWSim)
