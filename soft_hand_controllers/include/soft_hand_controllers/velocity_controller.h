#ifndef SOFT_HAND_CONTROLLERS__VELOCITY_CONTROLLER_H
#define SOFT_HAND_CONTROLLERS__VELOCITY_CONTROLLER_H

#include <std_msgs/Float64.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h> 

namespace soft_hand_controllers {

    class VelocityController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {

        public:
        
        // Default Constructor
        VelocityController();

        // Default Destructor
        ~VelocityController();

        // Controller functions
        bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time&);
		void command(const std_msgs::Float64::ConstPtr &msg);

        private:

        // A structure for joint limits of the soft hand
        struct limits_ {
			double min;
			double max;
			double center;
		} joint_limits_;

        // The joint name to be parsed from param server (from controllers.yaml)
        std::string joint_name_;

        // A URDF model of the robot and a joint
        urdf::Model urdf_;
        boost::shared_ptr<const urdf::Joint> urdf_joint_;

        // A joint handle
        hardware_interface::JointHandle joint_handle_;

        // Flag for position holding if cmd_value_ = 0
        int cmd_flag_;

        // Subscriber to the command topic of the controller
        ros::Subscriber cmd_sub_;
    
        // The current and commanded joint values (position and velocity)
        double curr_pos_;
        double old_pos_;
        double cmd_pos_;
        double cmd_vel_;

    };

}

#endif // SOFT_HAND_CONTROLLERS__VELOCITY_CONTROLLER_H