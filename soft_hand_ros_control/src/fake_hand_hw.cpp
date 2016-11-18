/** @file fake_hand_hw.cpp
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

#include "soft_hand_ros_control/fake_hand_hw.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(fake_hand_hw::FAKESH_HW, hardware_interface::RobotHW)

namespace fake_hand_hw {
    FAKESH_HW::FAKESH_HW() {}

    FAKESH_HW::~FAKESH_HW() {
        stop();
    }

    bool FAKESH_HW::init(ros::NodeHandle &n, ros::NodeHandle &robot_hw_nh) {
        nh_ = robot_hw_nh;

        return start();
    }

    bool FAKESH_HW::start() {
        // construct a new device (interface and state storage)
        device_ = std::make_shared<FAKESH_HW::FAKESH_device>();

        nh_.param("device_id", device_id_, 0);

        // TODO: use transmission configuration to get names directly from the URDF model
        if (ros::param::get("/iit_hand/joints", this->device_->joint_names)) {
            if (this->device_->joint_names.size() != N_SYN) {
                ROS_ERROR("This robot has 1 joint, you must specify 1 name only until more synergies are not included");
            }
        }
        else {
            ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
            throw std::runtime_error("No joint name specification");
        }
        if (!urdf_model_.initParam("robot_description")) {
            ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
            throw std::runtime_error("No URDF model available");
        }

        // initialize and set to zero the state and command values
        device_->init();
        device_->reset();

        // general joint to store information
        boost::shared_ptr<const urdf::Joint> joint;

        // create joint handles given the list
        for (unsigned int i = 0; i < N_SYN; ++i) {
            ROS_INFO_STREAM("Handling joint: " << this->device_->joint_names[i]);

            // get current joint configuration
            joint = urdf_model_.getJoint(this->device_->joint_names[i]);
            if (!joint.get()) {
                ROS_ERROR_STREAM("The specified joint "
                                 << this->device_->joint_names[i]
                                 << " can't be found in the URDF model. Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
                throw std::runtime_error("Wrong joint name specification");
            }

            // joint state handle
            hardware_interface::JointStateHandle state_handle(this->device_->joint_names[i],
                                                              &this->device_->joint_position[i],
                                                              &this->device_->joint_velocity[i],
                                                              &this->device_->joint_effort[i]);

            state_interface_.registerHandle(state_handle);

            // effort command handle
            hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
                        state_interface_.getHandle(this->device_->joint_names[i]),
                        &this->device_->joint_position_command[i]);

            position_interface_.registerHandle(joint_handle);

            registerJointLimits(this->device_->joint_names[i],
                                joint_handle,
                                &urdf_model_,
                                &this->device_->joint_lower_limits[i],
                                &this->device_->joint_upper_limits[i],
                                &this->device_->joint_effort_limits[i]);
        }

        ROS_INFO("Register state and position interfaces");

        // register ros-controls interfaces
        registerInterface(&state_interface_);
        registerInterface(&position_interface_);

        return true;
    }

    void FAKESH_HW::stop() {
        usleep(2000000);
    }

    void FAKESH_HW::read(const ros::Time &time, const ros::Duration &period) {
        static short int currents[2];

        ROS_DEBUG("Reading fake hand joint values");

        // fill the state variables
        for (unsigned int i = 0; i < N_SYN; i++) {
            this->device_->joint_position_prev[i] = device_->joint_position[i]/17000.0;
            this->device_->joint_position[i] = double(inputs_[0]) / 17000.0;
            this->device_->joint_effort[i] = double(currents[0]) * 1.0;
            this->device_->joint_velocity[i] =
                    filters::exponentialSmoothing((device_->joint_position[i] - device_->joint_position_prev[i]) / period.toSec(),
                                                  device_->joint_velocity[i], 0.2);
        }
    }

    void FAKESH_HW::write(const ros::Time &time, const ros::Duration &period) {
        ROS_DEBUG("Writing fake hand joint values");

        // enforce limits
        pj_sat_interface_.enforceLimits(period);
        pj_limits_interface_.enforceLimits(period);

        // write to the hand
        auto pos = short(17000.0 * device_->joint_position_command[0]);
        set_input(pos);
    }

    void FAKESH_HW::registerJointLimits(const std::string &joint_name,
                                        const hardware_interface::JointHandle &joint_handle,
                                        const urdf::Model * const urdf_model,
                                        double * const lower_limit,
                                        double * const upper_limit,
                                        double * const effort_limit) {

        *lower_limit = -std::numeric_limits<double>::max();
        *upper_limit = std::numeric_limits<double>::max();
        *effort_limit = std::numeric_limits<double>::max();

        joint_limits_interface::JointLimits limits;
        bool has_limits = false;
        joint_limits_interface::SoftJointLimits soft_limits;
        bool has_soft_limits = false;

        if (urdf_model != NULL) {
            const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);

            if (urdf_joint != NULL) {
                // Get limits from the URDF file.
                if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                    has_limits = true;

                if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                    has_soft_limits = true;
            }
        }

        if (!has_limits)
            return;

        if (limits.has_position_limits) {
            *lower_limit = limits.min_position;
            *upper_limit = limits.max_position;
        }
        if (limits.has_effort_limits)
            *effort_limit = limits.max_effort;

        if (has_soft_limits) {
            const joint_limits_interface::EffortJointSoftLimitsHandle
                    limits_handle(joint_handle, limits, soft_limits);
            ej_limits_interface_.registerHandle(limits_handle);
        }
        else {
            const joint_limits_interface::EffortJointSaturationHandle
                    sat_handle(joint_handle, limits);
            ej_sat_interface_.registerHandle(sat_handle);
        }
    }

    void FAKESH_HW::set_input(short pos) {
        inputs_[0] = pos;
        inputs_[1] = 0;
    }
}
