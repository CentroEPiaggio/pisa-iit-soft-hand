/** @file fake_hand_hw.h
 *  @brief A C++ class which implements a ros_control driver faking the Pisa/IIT
 * softhand.
 *  @author Murilo Martins (murilo.martins@ocado.com)
 *
 * Copyright (c) 2016, Ocado Technology
 * All rights reserved.
 *
 * This file is part of soft_hand_ros_control:
 * https://github.com/CentroEPiaggio/pisa-iit-soft-hand/tree/master/soft_hand_ros_control
 *
 * soft_hand_ros_control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.

 * soft_hand_ros_control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with soft_hand_ros_control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <memory>

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

// just thinking of any time in the future the soft hand might have more than one synergy
constexpr unsigned int n_syn = 1;

namespace fake_hand_hw {
class FakeHandHW: public hardware_interface::RobotHW {
public:
    /**
     * @brief Constructor.
     */
    FakeHandHW();
    /**
     * @brief Destructor.
     */
    virtual ~FakeHandHW();
    /**
     * @brief Method implementing all the initialisation required by the class.
     * @param unused node handle
     * @param robot (hand) hardware-specific node handle
     * @return True if initialisation was successful; False otherwise.
     */
    virtual bool init(ros::NodeHandle& n, ros::NodeHandle& robot_hw_nh);

    virtual void read(const ros::Time& time, const ros::Duration& period);
    virtual void write(const ros::Time& time, const ros::Duration& period);

    void registerJointLimits(const std::string& joint_name,
                             const hardware_interface::JointHandle& joint_handle,
                             const urdf::Model * const urdf_model,
                             double * const lower_limit,
                             double * const upper_limit,
                             double * const effort_limit);

    void set_input(short int pos);

    struct FakeHand_device {
        std::vector<std::string> joint_names;

        std::vector<double>
        joint_upper_limits,
        joint_lower_limits,
        joint_effort_limits;

        std::vector<double>
        joint_position,
        joint_position_prev,
        joint_velocity,
        joint_effort,
        joint_position_command;

        void init() {
            joint_position.resize(n_syn);
            joint_position_prev.resize(n_syn);
            joint_velocity.resize(n_syn);
            joint_effort.resize(n_syn);
            joint_position_command.resize(n_syn);

            joint_lower_limits.resize(n_syn);
            joint_upper_limits.resize(n_syn);
            joint_effort_limits.resize(n_syn);
        }

        void reset() {
            for (unsigned int i = 0; i < n_syn; ++i) {
                joint_position[i] = 0.0;
                joint_position_prev[i] = 0.0;
                joint_velocity[i] = 0.0;
                joint_effort[i] = 0.0;
                joint_position_command[i] = 0.0;
            }
        }
    };
private:
    std::shared_ptr<FakeHandHW::FakeHand_device> device_;

    ros::NodeHandle nh_;

    int device_id_;

    short int inputs_[2];

    urdf::Model urdf_model_;

    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::PositionJointInterface position_interface_;

    joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
    joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface_;

    bool start();
    void stop();
};
}
