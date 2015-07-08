// System
#include <iostream>
#include <string>
#include <thread>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <sensor_msgs/JointState.h>

#include <gazebo_msgs/ModelState.h>

// Interactive Grasping Simulator
class IGS
{
public:
    IGS():server_("interactive_grasping_simulator_interactive_marker")
    {
        hand_sub_ = node_.subscribe("interactive_grasping_simulator_hand", 10, &IGS::update_position, this);
        hand_marker_pub_ = node_.advertise<visualization_msgs::Marker>( "interactive_grasping_simulator_hand", 0 );
        hand_gazebo_pub_ = node_.advertise<gazebo_msgs::ModelState>( "/gazebo/set_model_state", 0 );

        im_sub_hand_ = node_.subscribe("interactive_grasping_simulator_interactive_marker/feedback",1,&IGS::im_callback,this);
        js_sub_ = node_.subscribe("/soft_hand/joint_states", 1, &IGS::publishTF, this);

        transform_.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform_.setRotation( tf::Quaternion( 0.0, 0.0, 0.0, 1.0) );

        node_.getParam("leftness", leftness_);

        // init pose at 0.5
        current_pose_.position.z = 0.5;

        // init empty-mesh marker
        marker_.header.frame_id = "world";
        marker_.lifetime=ros::DURATION_MAX;
        marker_.scale.x = 0.001;
        marker_.scale.y = 0.001;
        marker_.scale.z = 0.001;
        marker_.id = 1;
        marker_.ns = "soft_hand";
        marker_.color.a = 1;
        marker_.color.r = 1;
        marker_.color.g = 0;
        marker_.color.b = 0;
        marker_.pose = current_pose_;

        // init gazebo msg
        hand_in_gazebo_.model_name = "soft_hand";
        hand_in_gazebo_.reference_frame = "world";
    }
    ~IGS(){}

    void publish_hand_marker()
    {
        marker_.pose = current_pose_;
        hand_marker_pub_.publish(marker_);
    }
    
private:
    void update_position(const visualization_msgs::Marker &marker)
    {
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.name = "soft_hand";
        int_marker.description = "";
        int_marker.scale=0.2;

        int_marker.controls.clear();

        visualization_msgs::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back( marker);

        int_marker.controls.push_back( box_control );

        visualization_msgs::InteractiveMarkerControl control;
        
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
        
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
        
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
        
        int_marker.pose = marker.pose;

        server_.insert(int_marker);

        server_.applyChanges();
    }

    void im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback)
    {
        /*static tf::TransformListener tf;
        tf::StampedTransform hand_palm;
        double timeout = 5.0;

        if(!tf.waitForTransform("gmu_hand_palm_link","hand",ros::Time(0), ros::Duration(timeout)))
            hand_palm.setIdentity();
        else
            tf.lookupTransform("gmu_hand_palm_link","hand", ros::Time(0), hand_palm);*/

        transform_.setOrigin( tf::Vector3(feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z) );
        transform_.setRotation( tf::Quaternion( feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w) );
        /*transform_.mult(transform_, hand_palm);*/

        current_pose_ = feedback.pose;

        hand_in_gazebo_.pose = feedback.pose;
    }

    void publishTF(const sensor_msgs::JointState &msg)
    {
        tf_broadcaster_.sendTransform( tf::StampedTransform(transform_, ros::Time::now(), "world", "hand") );
        hand_gazebo_pub_.publish( hand_in_gazebo_ );
    }
    
    interactive_markers::InteractiveMarkerServer server_;

    ros::NodeHandle node_;    

    ros::Subscriber hand_sub_;

    geometry_msgs::Pose current_pose_;
    ros::Publisher hand_marker_pub_;
    ros::Publisher hand_gazebo_pub_;

    ros::Subscriber im_sub_hand_;
    ros::Subscriber js_sub_;
    
    tf::Transform transform_;
    tf::TransformBroadcaster tf_broadcaster_;

    gazebo_msgs::ModelState hand_in_gazebo_;
    visualization_msgs::Marker marker_;

    bool leftness_;
};

int main(int argc, char** argv)
{
    ros::init( argc, argv, "Interactive_Grasping_Simulator", ros::init_options::AnonymousName );
    
    ros::AsyncSpinner spin(1);
    spin.start();
    IGS igs;

    ROS_INFO_STREAM("This is a utility to simulate grasps using the soft hand");

    igs.publish_hand_marker();

    ROS_INFO_STREAM("Type any key when you are done");
    std::string mode;
    std::cin >> mode;

    ROS_INFO_STREAM("Bye! Thanks for using this terrific program.");
    return 0;
}
