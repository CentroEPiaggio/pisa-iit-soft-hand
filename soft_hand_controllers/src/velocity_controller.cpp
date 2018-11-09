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
        // Debug message
        ROS_DEBUG("Entered init function of soft_hand_controllers VelocityController.");

        // Getting the joint name from the parameter server
        if(!n.getParam("joint", this->joint_name_)){
            ROS_ERROR("Joint name not found in namespace: %s.", n.getNamespace().c_str());
            return false;
        }

        // Construct an URDF model from the xml string
        std::string model_string;

        if (n.hasParam("robot_description")){
            n.getParam("robot_description", model_string);
        } else {
            ROS_ERROR("Parameter robot_description not set, shutting controller.");
            return false;
        }

        // Parsing the URDF model
        if(!this->urdf_.initString(model_string)){
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

        // Debug message
        ROS_DEBUG("Sucessfully initialized the joint handle for joint '%s'", this->joint_name_.c_str());

        // Getting and setting the joint limits
        this->joint_limits_.min = this->urdf_joint_->limits->lower;
        this->joint_limits_.max = this->urdf_joint_->limits->upper;
        this->joint_limits_.center = (this->joint_limits_.max + this->joint_limits_.min) / 2;

        // Debug message
        ROS_DEBUG("The joint limits are: lower = %f, upper = %f.", this->joint_limits_.min, this->joint_limits_.max);

        // Initializing the cmd_flag_
        this->cmd_flag_ = 0;

        // Initializing subscriber to command topic
        this->cmd_sub_ = n.subscribe<std_msgs::Float64>("command", 1, &VelocityController::command, this);

        return true;
    }

    // STARTING FUNCTION
	void VelocityController::starting(const ros::Time& time){
        // Setting desired twist to zero
        this->cmd_vel_ = 0.0;

        // Reading the joint state (position) and setting command
        this->curr_pos_ = this->joint_handle_.getPosition();
        this->old_pos_ = this->joint_handle_.getPosition();
        this->cmd_pos_ = this->curr_pos_;
        this->joint_handle_.setCommand(this->cmd_pos_);
    }

    // UPDATING FUNCTION
	void VelocityController::update(const ros::Time& time, const ros::Duration& period){
        // THE CODE INSIDE NEXT IF EXECUTED ONLY IF cmd_value_ != 0
        if(this->cmd_flag_){
            // Reading the joint state
            this->curr_pos_ = this->joint_handle_.getPosition();

            // Debug message
            ROS_DEBUG_STREAM("The measured synergy velocity  = " << this->joint_handle_.getVelocity() << ".");
            ROS_DEBUG("The previous joint command  = %f.", this->cmd_pos_);
            ROS_DEBUG("The old joint position  = %f.", this->old_pos_);
            ROS_DEBUG("The current joint position = %f.", this->curr_pos_);
            ROS_DEBUG("The achieved joint speed  = %f.", (this->curr_pos_ - this->old_pos_) / period.toSec());
            ROS_DEBUG("The current commanded vel = %f.", this->cmd_vel_);
            ROS_DEBUG("The current dt = %f.", period.toSec());

            // Setting command to current position
            this->cmd_pos_ = this->curr_pos_;

            // Integrating the speed to find the next position (Forward Euler)
            this->cmd_pos_ += this->cmd_vel_ * period.toSec();

            // Debug message
            ROS_DEBUG("The current joint command before saturation = %f.", this->cmd_pos_);

            // Checking if the computed position is within limits
            if(this->cmd_pos_ < this->joint_limits_.min){
                this->cmd_pos_ = this->joint_limits_.min;
            }
            if(this->cmd_pos_ > this->joint_limits_.max){
                this->cmd_pos_ = this->joint_limits_.max;
            }

            this->old_pos_ = this->curr_pos_;

            // Debug message
            ROS_DEBUG("The current joint command after saturation = %f.", this->cmd_pos_);

            // Debug to check execution frequency
            ROS_DEBUG_STREAM("VelocityController: Executing Joint Speed Command!!!");
        } // end if(cmd_flag_)

        // Sending the command to the position interface
        this->joint_handle_.setCommand(this->cmd_pos_);
    }

    // COMMAND SUBSCRIBER CALLBACK
	void VelocityController::command(const std_msgs::Float64::ConstPtr &msg){
        // Saving the msg to cmd_value_
        this->cmd_vel_ = double (msg->data);

        // Setting cmd_flag_ according to the commanded value
        if(this->cmd_vel_ == 0.0) this->cmd_flag_ = 0;
        else this->cmd_flag_ = 1;
    }

}

PLUGINLIB_EXPORT_CLASS(soft_hand_controllers::VelocityController, controller_interface::ControllerBase)