// setup
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <random>
#include <cmath>
#include <chrono>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

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

void setposegoal(moveit::planning_interface::MoveGroupInterface *group, moveit::planning_interface::PlanningSceneInterface *scene, robot_state::RobotState *state, robot_model_loader::RobotModelLoader *model)
{
  const moveit::core::JointModelGroup *joint_model_group =
      group->getCurrentState()->getJointModelGroup("manipulator");

  // Set up random number generators
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> uniform01(0.4, 1.0);
  std::uniform_real_distribution<double> uniform02(-1.0, 1.0);
  // generate N random numbers
  int N = 1000;

  // the correct way
  FILE *correct;
  correct = fopen("correct.csv", "w");
  fprintf(correct, "Theta,Phi,x,y,z\n");
  // for (int i = 0; i < N; i++) {
  double radius = 0.3;
  double theta = 0.3 * M_PI * uniform02(generator);
  double phi = -0.3 * M_PI * uniform01(generator);
  double x = radius * sin(phi) * cos(theta);
  double y = radius * sin(phi) * sin(theta);
  double z = radius * cos(phi);

  // fprintf(correct, "%f,%f,%f,%f,%f\n", theta, phi, x, y, z);

  // double Point_marker[3][1]={x,y,z};
  // double roll[3][3]={{1,0,0},{0,0,1},{0,-1,0}};
  // double pitch[3][3]={{0,-1,0},{1,0,0},{0,0,1}};
  // double Point_world[3][1]={-y,z,-x};
  // double yawfororientation[3][3]={{cos(theta),-sin(theta),0},{sin(theta),cos(theta),0},{0,0,1}};
  // double pitchfororientation[3][3]={{cos(phi),0,sin(phi)},{0,1,0},{-sin(phi),0,cos(phi)}};
  // double Point_orientation[3][3] = {{sin(theta),cos(theta)*sin(phi),-cos(theta)*cos(phi)},{-cos(theta),sin(theta)*sin(phi),-cos(phi)*sin(theta)},{0,cos(phi),sin(phi)}};

  // 2d vector example
  // std::vector<std::vector<double>> matrix;

  // eigen cpp
  Eigen::Matrix3d roll, pitch, yaw, convert, rot, mtoh;
  Eigen::Matrix4d handtomarker, markertobase, handtobase, oldhtom;
  yaw << cos(theta), -sin(theta), 0,
      sin(theta), cos(theta), 0,
      0, 0, 1;
  pitch << cos(phi), 0, sin(phi),
      0, 1, 0,
      -sin(phi), 0, cos(phi);
  mtoh << 0, 1, 0,
      1, 0, 0,
      0, 0, -1;
  oldhtom << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 1;
  oldhtom.topLeftCorner(3, 3) = mtoh.inverse();
  rot = yaw * pitch;
  markertobase << 1, 0, 0, 0.8,
      0, 1, 0, 0,
      0, 0, 1, 0.3,
      0, 0, 0, 1;
  handtomarker << 0, 0, 0, x,
      0, 0, 0, y,
      0, 0, 0, z,
      0, 0, 0, 1;
  handtomarker.topLeftCorner(3, 3) = rot;
  std::cout << markertobase << std::endl;
  std::cout << handtomarker << std::endl;

  handtobase = markertobase * handtomarker * oldhtom;
  convert = handtobase.topLeftCorner(3, 3);
  std::cout << handtobase << std::endl;
  std::cout << handtomarker << std::endl;

  Eigen::Quaterniond q;
  q = convert;

  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.x = q.x();
  start_pose2.orientation.y = q.y();
  start_pose2.orientation.z = q.z();
  start_pose2.orientation.w = q.w();
  start_pose2.position.x = handtobase(0, 3);
  start_pose2.position.y = handtobase(1, 3);
  start_pose2.position.z = handtobase(2, 3);
  std::cout << start_pose2 << std::endl;

  group->setPoseTarget(start_pose2);

  Eigen::Affine3d affine;
  affine = handtobase;
  tf::Transform transform;
  tf::poseEigenToTF(affine, transform);
  static tf::TransformBroadcaster br;
  std::string sampled_hand = "sampled_hand";
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", sampled_hand));

  // state->setFromIK(joint_model_group, start_pose2);
  // std::vector<double> joint_values;

  // state->copyJointGroupPositions(joint_model_group, joint_values);

  // std::cout << "Joint value:";
  // for (auto j : joint_values)
  //   std::cout << j << ",";
  // std::cout << std::endl;

  // sensor_msgs::JointState joint_state;
  // joint_state.position.resize(6);
  // joint_state.name.resize(6);
  // joint_state.name = joint_model_group->getVariableNames();

  // joint_state.name[1] = "shoulder_lift_joint";
  // joint_state.name[2] = "elbow_joint";
  // joint_state.name[3] = "wrist_1_joint";
  // joint_state.name[4] = "wrist_2_joint";
  // joint_state.name[5] = "wrist_3_joint";

  // std::vector<double>
  //     joint_group_positions;
  // for (int i = 0; i < 6; i++)
  // {
  //   joint_group_positions.push_back(0.0);
  // }
  // state->setVariablePositions(joint_state.name, joint_values);
  // // group->setStartState(state(model->getModel()));
  // std::cout << "Joint names:";
  // for (auto n : joint_model_group->getJointModelNames())
  //   std::cout << n << ",";
  // std::cout << std::endl;

  // joint_group_positions = {0, -M_PI / 2, 0.0, 0.0, -1.5708, 1.06041};
  // group->setJointValueTarget(joint_values);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampleforrange");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  // Load the robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  // Create a RobotState instance
  robot_state::RobotState start_state(kinematic_model);

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup *joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

  ROS_INFO_NAMED("ur5", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("ur5", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("ur5", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Define rosbag
  rosbag::Bag bag;
  bag.open("aruco_joints.bag", rosbag::bagmode::Write);

  // then loop your sample collection

  // surpose you want to collect 15 good samples for calibration.
  int counts = 0;
  while (counts < 15)
  {
    // 1. sample
    setposegoal(&move_group_interface, &planning_scene_interface, &start_state, &robot_model_loader);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // 2. motion planning
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("ur5", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success = true)
    {

      // visualise the paln path

      Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
      text_pose.translation().z() = 1.0;
      visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
      ROS_INFO_NAMED("ur5", "Visualizing plan 1 as trajectory line");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

      // 3.execution
      move_group_interface.execute(my_plan);

      // if everything ok (target achieved, good image, collection of end-effector pose, joint_states)
    }
    ROS_INFO("%d samples have been collected", counts);
    char ch = std::cin.get();
    if (ch == 'y' || ch == 'Y')
    {
      // c1. This is to wait for the image topic for your marker
      sensor_msgs::Image::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Image>("ur5/camera1/image_raw", nh);
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // Save the image

      geometry_msgs::Pose pose_msg;
      // c2. Following is to collect the pose of robot end-effector, with respect to robot base
      tf::StampedTransform transform;
      tf::TransformListener listener;
      bool tf_collected = false;
      while (!tf_collected)
      {
        try
        {
          // Wait for the transform to become available
          listener.waitForTransform("base_link", "tool0", ros::Time(0), ros::Duration(1.0));

          // Get the transform from "source_frame" to "target_frame"
          listener.lookupTransform("base_link", "tool0", ros::Time(0), transform);

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
      // c3. This is to collect joint state info for current sampling robot pose
      sensor_msgs::JointState::ConstPtr jmsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);
      // Save the image
      cv::imwrite("aruco_image.jpg", cv_ptr->image);

      // c4. Write the message to the bag file for this sample
      bag.write("/3d_image/image_raw", ros::Time::now(), *msg);
      bag.write("/joint_states", ros::Time::now(), *jmsg);
      bag.write("/endpose_in_base", ros::Time::now(), pose_msg);

      counts++;
    }
  }
  bag.close();
  return 0;
}
