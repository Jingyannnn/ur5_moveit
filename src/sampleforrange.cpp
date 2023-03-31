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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampleforrange");

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  geometry_msgs::Pose target_pose1;
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

  // Create a RobotState instance
  robot_state::RobotState target_state(*move_group_interface.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.35;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  target_state.setFromIK(joint_model_group, start_pose2);
  std::vector<double> joint_group_positions;
  target_state.copyJointGroupPositions(joint_model_group, joint_group_positions);
  move_group_interface.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("ur5", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface.execute(my_plan);
  // visualise the paln path

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  ROS_INFO_NAMED("ur5", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // execute
  moveit_msgs::RobotTrajectory trajectory;
  move_group_interface.execute(trajectory);

  return 0;
}