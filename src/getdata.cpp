
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char **argv)
{
//Define rosbag
rosbag::Bag bag;
bag.open("aruco_joints.bag", rosbag::bagmode::Write);

//then loop your sample collection

ros::init(argc, argv, "wait_message");
ros::NodeHandle nh;
ros::Rate loop_rate(1);

//1. This is to wait for the image topic for your marker 
sensor_msgs::Image::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Image>("ur5/camera_1/image_raw", nh);
cv_bridge::CvImagePtr cv_ptr;
cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
// Save the image

geometry_msgs::Pose pose_msg;
//2. Following is to collect the pose of robot end-effector, with respect to robot base
tf::StampedTransform transform;
tf::TransformListener listener;
bool tf_collected = false;
while (!tf_collected)
{
    try
    {
        // Wait for the transform to become available
        listener.waitForTransform("base_link", "tool_0", ros::Time(0), ros::Duration(1.0));

        // Get the transform from "source_frame" to "target_frame"
        listener.lookupTransform("base_link", "tool_0", ros::Time(0), transform);

        tf::Vector3 translation = transform.getOrigin();
        tf::Quaternion rotation = transform.getRotation();

        pose_msg.position.x = translation.getX();
        pose_msg.position.y = translation.getY();
        pose_msg.position.z = translation.getZ();

        pose_msg.orientation.x = rotation.getX();
        pose_msg.orientation.y = rotation.getY();
        pose_msg.orientation.z = rotation.getZ();
        pose_msg.orientation.w = rotation.getW();
        tf_collected = true;
        }
        catch (tf::TransformException ex)
        {
        // If the transform lookup fails, print an error message
        ROS_ERROR("%s", ex.what());
    }
}
//3. This is to collect joint state info for current sampling robot pose
sensor_msgs::JointState::ConstPtr jmsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);
// Save the image
cv::imwrite("aruco_image.jpg", cv_ptr->image);


// 4. Write the message to the bag file for this sample
bag.write("/3d_image/image_raw", ros::Time::now(), *msg);
bag.write("/joint_states", ros::Time::now(), *jmsg);
bag.write("/endpose_in_base", ros::Time::now(), pose_msg);

rosbag::View view(bag);
std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
std::set<std::string> topics;

BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) {
  topics.insert(info->topic);
}

return 0;
}