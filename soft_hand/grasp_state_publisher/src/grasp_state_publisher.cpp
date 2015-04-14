#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SoftHand grasp observer");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ROS_INFO("The hand might be grasping an object or not");

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}