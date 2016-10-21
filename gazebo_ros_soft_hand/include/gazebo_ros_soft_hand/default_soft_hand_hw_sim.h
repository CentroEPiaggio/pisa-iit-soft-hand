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

   Author: Carlos Rosales, Hamal Marino
   Desc: Not any simulated robot, this is the soft hand hardware interface for simulation
*/

#ifndef _GAZEBO_ROS_SOFT_HAND___DEFAULT_SOFT_HAND_HW_SIM_H_
#define _GAZEBO_ROS_SOFT_HAND___DEFAULT_SOFT_HAND_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// you need the forked version of ros_control to use this version of the SoftHand in simulation
// git clone https://github.com/CentroEPiaggio/ros_control.git
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_loader.h>
#include <adaptive_transmission/adaptive_synergy_transmission.h>
#include <adaptive_transmission/loader_utils.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ContactSensor.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Wrench.h>

// gazebo_ros_soft_hand
#include <gazebo_ros_soft_hand/soft_hand_hw_sim.h>

// URDF
#include <urdf/model.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// Eigen
#include <eigen3/Eigen/Core>

namespace gazebo_ros_soft_hand
{

class DefaultSoftHandHWSim : public gazebo_ros_soft_hand::SoftHandHWSim
{
public:

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

protected:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           double *const lower_limit, double *const upper_limit, 
                           double *const effort_limit);

  unsigned int n_dof_;
  unsigned int n_mimic_;
  unsigned int n_links_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  // synergy state controlled by standard method
  std::string synergy_name_;
  double synergy_position_;
  double synergy_velocity_;
  double synergy_effort_;
  double synergy_effort_command_;
  double synergy_position_command_;
  double synergy_velocity_command_;
  double synergy_lower_limit_;
  double synergy_upper_limit_;
  double synergy_effort_limit_;
  ControlMethod synergy_control_method_;
  control_toolbox::Pid pid_controller_;

  gazebo::physics::JointPtr sim_synergy_;

  // joint states controlled through the transmission
  std::vector<std::string> link_names_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_contact_; // the effort generated by the contact J^T*f_c
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;
  std::vector<gazebo::physics::LinkPtr> sim_links_;
  std::vector<gazebo::sensors::ContactSensorPtr> sim_sensors_;

  // joint mimic states which are a mirror of the joints above
  std::vector<std::string> joint_names_mimic_;
  std::vector<double> joint_position_mimic_;
  std::vector<double> joint_velocity_mimic_;
  std::vector<double> joint_effort_mimic_;
  std::vector<double> joint_effort_command_mimic_;
  std::vector<double> joint_position_command_mimic_;
  std::vector<double> joint_velocity_command_mimic_;

  std::vector<gazebo::physics::JointPtr> sim_joints_mimic_;

  // adaptive transmission only, the simple transmission of the synergy joint
  // is not used so far, but it could be a way to model motor dynamics later
  adaptive_transmission_interface::AdaptiveSynergyTransmission* adaptive_trans_;

  // transmission interfaces
  transmission_interface::JointToActuatorEffortInterface jnt_to_act_eff_;
  transmission_interface::JointToActuatorPositionInterface jnt_to_act_pos_;
  transmission_interface::ActuatorToJointEffortInterface act_to_jnt_eff_;
  transmission_interface::ActuatorToJointPositionInterface act_to_jnt_pos_;
  transmission_interface::ActuatorData a_state_data_[2];
  transmission_interface::ActuatorData a_cmd_data_[2];
  transmission_interface::JointData j_state_data_[2];
  transmission_interface::JointData j_cmd_data_[2];

  // contact information per link
  std::vector<gazebo_msgs::ContactsState> contacts_;
  std::vector<ros::Subscriber> sub_contacts_;

  std::map<std::string, KDL::Wrench> link_applied_wrench_;

  void getContacts(const gazebo_msgs::ContactsState & msg);

  // kinematic chains
  KDL::Tree hand_tree_;

  // ToDo: implement this a std::map with link names, or with templates
  KDL::JntArray thumb_to_knuckle_joints_; // only the abduction, no mimic
  KDL::JntArray thumb_to_proximal_joints_; // abduction + inner + mimic
  KDL::JntArray thumb_to_distal_joints_; // abduction + inner + outer + mimic
  KDL::Chain thumb_to_knuckle_chain_;
  KDL::Chain thumb_to_proximal_chain_;
  KDL::Chain thumb_to_distal_chain_;
  KDL::ChainJntToJacSolver* thumb_to_knuckle_jac_solver_;
  KDL::ChainJntToJacSolver* thumb_to_proximal_jac_solver_;
  KDL::ChainJntToJacSolver* thumb_to_distal_jac_solver_;
  KDL::ChainFkSolverPos_recursive* thumb_to_knuckle_fk_solver_;
  KDL::ChainFkSolverPos_recursive* thumb_to_proximal_fk_solver_;
  KDL::ChainFkSolverPos_recursive* thumb_to_distal_fk_solver_;
  KDL::Jacobian thumb_to_knuckle_jac_;
  KDL::Jacobian thumb_to_knuckle_jac_6x5_;
  KDL::Jacobian thumb_to_proximal_jac_;
  KDL::Jacobian thumb_to_proximal_jac_6x5_;
  KDL::Jacobian thumb_to_distal_jac_;
  KDL::Frame thumb_knuckle_frame_;
  KDL::Frame thumb_proximal_frame_;
  KDL::Frame thumb_distal_frame_;

  unsigned int n_fingers_;
  std::vector<KDL::JntArray> fingers_to_knuckle_joints_; // only the abduction, no mimic
  std::vector<KDL::JntArray> fingers_to_proximal_joints_; // abduction + inner + mimic
  std::vector<KDL::JntArray> fingers_to_middle_joints_; // abduction + inner + middle + mimic
  std::vector<KDL::JntArray> fingers_to_distal_joints_; // abduction + inner + middle + outer + mimic
  std::vector<KDL::Chain> fingers_to_knuckle_chain_;
  std::vector<KDL::Chain> fingers_to_proximal_chain_;
  std::vector<KDL::Chain> fingers_to_middle_chain_;
  std::vector<KDL::Chain> fingers_to_distal_chain_;
  std::vector<KDL::ChainJntToJacSolver*> fingers_to_knuckle_jac_solver_;
  std::vector<KDL::ChainJntToJacSolver*> fingers_to_proximal_jac_solver_;
  std::vector<KDL::ChainJntToJacSolver*> fingers_to_middle_jac_solver_;
  std::vector<KDL::ChainJntToJacSolver*> fingers_to_distal_jac_solver_;
  std::vector<KDL::ChainFkSolverPos_recursive*> fingers_to_knuckle_fk_solver_;
  std::vector<KDL::ChainFkSolverPos_recursive*> fingers_to_proximal_fk_solver_;
  std::vector<KDL::ChainFkSolverPos_recursive*> fingers_to_middle_fk_solver_;
  std::vector<KDL::ChainFkSolverPos_recursive*> fingers_to_distal_fk_solver_;
  std::vector<KDL::Jacobian> fingers_to_knuckle_jac_;
  std::vector<KDL::Jacobian> fingers_to_knuckle_jac_6x7_;
  std::vector<KDL::Jacobian> fingers_to_proximal_jac_;
  std::vector<KDL::Jacobian> fingers_to_proximal_jac_6x7_;
  std::vector<KDL::Jacobian> fingers_to_middle_jac_;
  std::vector<KDL::Jacobian> fingers_to_middle_jac_6x7_;
  std::vector<KDL::Jacobian> fingers_to_distal_jac_;
  std::vector<KDL::Frame> fingers_knuckle_frame_;
  std::vector<KDL::Frame> fingers_proximal_frame_;
  std::vector<KDL::Frame> fingers_middle_frame_;
  std::vector<KDL::Frame> fingers_distal_frame_;

  void updateKinematics();
  void updateStatics();

  void MultiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest);

};

typedef boost::shared_ptr<DefaultSoftHandHWSim> DefaultSoftHandHWSimPtr;

}

#endif // #ifndef __GAZEBO_ROS_SOFT_HAND_PLUGIN_DEFAULT_SOFT_HAND_HW_SIM_H_
