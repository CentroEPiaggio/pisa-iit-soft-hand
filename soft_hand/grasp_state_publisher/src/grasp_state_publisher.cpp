#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/JointState.h>

// bfl
#include <bfl/pdf/gaussian.h>

// generated header from msg/GraspState.msg
#include "grasp_state_publisher/GraspState.h"

// class of grasp observer
#include "grasp_state_publisher/soft_grasp_observer.h"

namespace grasp_state_publisher
{

class GraspStatePublisher
{
private:
	// suggested members, note the member_name_ naming convention

	// the node handle
	ros::NodeHandle nh_;

	// node handle in the private namespace
	ros::NodeHandle priv_nh_;

	// subscribers
	message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states_sim_;
	message_filters::Subscriber<sensor_msgs::JointState> sub_joint_states_imu_;
	// message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_ft_measurement_;
	// message_filters::Subscriber<geometry_msgs::PoseStamped> sub_hand_object_relative_pose_;

	// sync-er
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState> SyncPolicy_;
	message_filters::Synchronizer<SyncPolicy_> sensor_sync_;

	// publishers
	ros::Publisher pub_grasp_states_;

    // grasp observer
  	SoftGraspObserver grasp_observer_;


	// grasp state message
	grasp_state_publisher::GraspState msg_grasp_state_;



public:
	// callback functions
	void estimateGraspState(const sensor_msgs::JointState::ConstPtr & sim_joints, const sensor_msgs::JointState::ConstPtr & imu_joints);
    void publishGraspState();

	// constructor
	GraspStatePublisher(ros::NodeHandle nh) : 
		nh_(nh), 
		priv_nh_("~"),
		// subscribe to topics
		sub_joint_states_sim_(nh, nh.resolveName("joint_states"), 4),
		sub_joint_states_imu_(nh, nh.resolveName("glove/joint_states"), 4),
		// create the sync-er, 4 is the queue size
		sensor_sync_(SyncPolicy_(4), sub_joint_states_sim_, sub_joint_states_imu_)
	{
		// register the callback function
		sensor_sync_.registerCallback(boost::bind(&GraspStatePublisher::estimateGraspState, this, _1, _2));

		// advertise topics
		pub_grasp_states_ = nh_.advertise<grasp_state_publisher::GraspState>(nh_.resolveName("grasp_state"), 10);
	
		// load parameters and init grasp observer
		std::vector<double> mu_grasp, var_grasp, mu_no_grasp, var_no_grasp;
		nh_.getParam("mu_grasp", mu_grasp);
		nh_.getParam("var_grasp", var_grasp);
		nh_.getParam("mu_no_grasp", mu_no_grasp);
		nh_.getParam("var_no_grasp", var_no_grasp);

		if ( mu_grasp.size() != var_grasp.size() )
			ROS_ERROR("Parameter vectors for grasp class must be of same length");

		if( mu_no_grasp.size() != var_no_grasp.size() )
			ROS_ERROR("Parameter vectors for no grasp class must be of same length");

		if( mu_grasp.size() != mu_no_grasp.size() )
			ROS_ERROR("Parameter vector between classes must be of same length");

		grasp_observer_.init(mu_grasp, var_grasp, mu_no_grasp, var_no_grasp);
	}

	//! Empty stub
	~GraspStatePublisher() {}

};

// the grouped callback function
void GraspStatePublisher::estimateGraspState(const sensor_msgs::JointState::ConstPtr & sim_joints, const sensor_msgs::JointState::ConstPtr & imu_joints)
{

	// control that the size of the message in the topics is the same as in the PDFs
	if ( grasp_observer_.getNumberOfVars() != sim_joints->position.size() )
	{
		ROS_FATAL("The internal model and input dimensions do not match!");
	}

	// control that sizes are the same for both joint states
	if( sim_joints->position.size() != imu_joints->position.size() )
	{
		ROS_ERROR("Message dropped, joint state sources of different sizes");
		return;
	}

    // compute the x vector as the differences in absolute values of joint positions
    // between a synergistic and measured model
    std::vector<double> x;
    x.resize(sim_joints->position.size());

    ROS_INFO("Updating input vector");

    for( int i = 0; i < sim_joints->position.size(); ++i)
    {
    	ROS_INFO("inside for loop");
        x.at(i) = std::abs ( sim_joints->position.at(i) - imu_joints->position.at(i) );
        std::cout << "input vector at " << i << ": " << x.at(i) << std::endl;
    }

    // estimate the grasp using the observer
    grasp_observer_.setX( x );

    ROS_INFO("Filling the message");

    msg_grasp_state_.header = sim_joints->header;
    msg_grasp_state_.grasp = grasp_observer_.isGrasping();
    msg_grasp_state_.grasp_confidence = grasp_observer_.getGraspConfidence();
    msg_grasp_state_.no_grasp_confidence = grasp_observer_.getNoGraspConfidence();

	return;
}

// just publish the internal grasp state
void GraspStatePublisher::publishGraspState()
{
	pub_grasp_states_.publish(msg_grasp_state_);
	return;
}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SoftHand grasp observer");

	ros::NodeHandle nh;

	grasp_state_publisher::GraspStatePublisher grasp_publisher(nh);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		//ROS_INFO("The hand might be grasping an object or not");

		grasp_publisher.publishGraspState();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
