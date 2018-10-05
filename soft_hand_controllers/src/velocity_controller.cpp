#include <soft_hand_controllers/velocity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace soft_hand_controllers {

    // DEFAULT CONSTRUCTOR
    VelocityController::VelocityController(){
        // Nothing to do
    }

    // DEFAULT DESTRUCTOR
    VelocityController::~VelocityController(){
        // Shutting down command subscriber
        cmd_sub_.shutdown();
    }

    // INITIALIZING FUNCTION
    bool VelocityController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n){
        // Getting the joint name from the parameter server
        if(!n.getParam("joint", this->joint_name_)){
            ROS_ERROR("Joint name not found in namespace: %s.", n.getNamespace().c_str());
            return false;
        }

        // Parsing the URDF model
        if(!this->urdf_.initParamWithNodeHandle("robot_description", n)){
            ROS_ERROR("Failed to get the URDF from robot description!");
            return false;
        }

        // Setting the URDF joint using the joint name and checking if it is present in the URDF model
        this->urdf_joint_ = this->urdf_.getJoint(this->joint_name_);
        if(!this->urdf_joint_){
            ROS_ERROR("Could not find joint '%s' in the URDF.", this->joint_name_.c_str());
        }

        // Getting joint handle from hardware interface
        this->joint_handle_ = robot->getHandle(this->joint_name_);

        // Getting and setting the joint limits
        this->joint_limits_.min = this->urdf_joint_->limits->lower;
        this->joint_limits_.max = this->urdf_joint_->limits->upper;
        this->joint_limits_.center = (this->joint_limits_.max + this->joint_limits_.min) / 2;

        // Initializing the cmd_flag_
        cmd_flag_ = 0;

        // Initializing subscriber to command topic
        this->cmd_sub_ = n.subscribe<std_msgs::Float64>("command", 1, &VelocityController::command, this);

        return true;
    }

    // STARTING FUNCTION
	void VelocityController::starting(const ros::Time& time){
        // Getting previous time
        last_time_ = time;

        // Setting desired twist to zero
        cmd_vel_ = 0.0;

        // Reading the joint state (position) and setting command
        this->curr_pos_ = this->joint_handle_.getPosition();
        this->cmd_pos_ = this->curr_pos_;
        this->joint_handle_.setCommand(cmd_pos_);
    }

    // UPDATING FUNCTION
	void VelocityController::update(const ros::Time& time, const ros::Duration& period){
        // THE CODE INSIDE NEXT IF EXECUTED ONLY IF cmd_value_ != 0
        if(cmd_flag_){
            // Getting current time resolution and updating last_time_
            current_time_ = time;
            dt_ = current_time_ - last_time_;
            last_time_ = current_time_;

            // Reading the joint state
            this->curr_pos_ = this->joint_handle_.getPosition();

            // Setting command to current position (if cmd_flag_ = 0, position hold)
            this->cmd_pos_ = this->curr_pos_;

            // Integrating the speed to find the next position (Forward Euler)
            this->cmd_pos_ += this->cmd_vel_ * dt_.toSec();

            // Checking if the computed position is within limits
            if(this->cmd_pos_ < joint_limits_.min){
                this->cmd_pos_ = joint_limits_.min;
            }
            if(this->cmd_pos_ > joint_limits_.max){
                this->cmd_pos_ = joint_limits_.max;
            }
        } // end if(cmd_flag_)

        // Sending the command to the position interface
        this->joint_handle_.setCommand(this->cmd_pos_);
    }

    // COMMAND SUBSCRIBER CALLBACK
	void VelocityController::command(const std_msgs::Float64::ConstPtr &msg){
        // Saving the msg to cmd_value_
        this->cmd_vel_ = double (msg->data);

        // Setting cmd_flag_ according to the commanded value
        if(cmd_vel_ == 0.0) cmd_flag_ = 0;
        else cmd_flag_ = 1;
    }

}