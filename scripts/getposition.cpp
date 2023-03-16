# This script is to transfer aruco/camera pose to 4*4 matrix and use tf listener to call tf camera/baselink
include "ros/ros.h"
include "std_msgs/String.h"

void Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  
  ros::Subscriber matrix = n.subscribe("ur5/camera1/image_raw",1000,Callback);
  
  ros::spin();

  return 0;
}