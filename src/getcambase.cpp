//use tf listener to call tf camera/baselink
// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <turtlesim/Spawn.h>


class myclass{
  using namespace std;
  using namespace tf2;

  void Callback(const ur5_moveit::Poses::ConstPtr& msg)
  {
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
      
      geometry_msgs::Orientation* geometry_q = &msg->Poses.pose.orientation;
      geometry_msgs::Position* geometry_p = &msg->Poses.pose.position;

      // cout << *geometry_q

      tf2::Quaternion tf2_q(geometry_q->x,geometry_q->y,geometry_q->z,geometry_q->w );
      tf2::Vector3 tf2_p(geometry_p->x,geometry_p->y,geometry_p->z);
          
  }

  int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle node;

    ros::Subscriber matrix = node.subscribe("arucotf",1,Callback);

    tf2::Transform::Transform matrix(const Quaternion& q, const Vector3& c = Vector3(tf2Scalar(0), tf2Scalar(0), tf2Scalar(0)));

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (node.ok()){
      geometry_msgs::TransformStamped transformStamped;
      try{
        cambasetf = tfBuffer.lookupTransform("camera_link", "base_link",
                                ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      rate.sleep();
      ros::spinOnce();
    }
    return 0;
};
};