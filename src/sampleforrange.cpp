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


void setposegoal(moveit::planning_interface::MoveGroupInterface* group, moveit::planning_interface::PlanningSceneInterface* scene,robot_state::RobotState* state, robot_model_loader::RobotModelLoader* model){
  const moveit::core::JointModelGroup *joint_model_group =
      group->getCurrentState()->getJointModelGroup("manipulator");
  
  // Set up random number generators
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator (seed);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);

    // generate N random numbers
    int N = 1000;

    // the correct way
    FILE * correct;
    correct = fopen("correct.csv", "w");
    fprintf(correct, "Theta,Phi,x,y,z\n");
    for (int i = 0; i < N; i++) {
        double radius= 0.5;
        double theta = 2 * M_PI * uniform01(generator);
        double phi = M_PI * uniform01(generator);
        double x = 0.5+radius*sin(phi) * cos(theta);
        double y = -0.5+ radius*sin(phi) * sin(theta);
        double z = 0.5+ radius*cos(phi);
        fprintf(correct, "%f,%f,%f,%f,%f\n", theta, phi, x, y, z);

        double Point_marker[3][1]={x,y,z};
        double roll[3][3]={{1,0,0},{0,0,1},{0,-1,0}};
        double pitch[3][3]={{0,-1,0},{1,0,0},{0,0,1}};
        double Point_world[3][1]={-y,z,-x};
        double rollfororientation[3][3]={{cos(theta),-sin(theta),0},{sin(theta),cos(theta),0},{0,0,1}};
        double pitchfororientation[3][3]={{cos(phi),0,sin(phi)},{0,1,0},{-sin(phi),0,cos(phi)}};
        double Point_orientation[3][3] = {{sin(theta),cos(theta)*sin(phi),-cos(theta)*cos(phi)},{-cos(theta),sin(theta)*sin(phi),-cos(phi)*sin(theta)},{0,cos(phi),sin(phi)}};

        float m00 = sin(theta);
        float m01 = cos(theta)*sin(phi);
        float m02 = -cos(theta)*cos(phi);
        float m10 = -cos(theta);
        float m11 = sin(theta)*sin(phi);
        float m12 = -cos(phi)*sin(theta);
        float m20 = 0;
        float m21 = cos(phi);
        float m22 = sin(phi);
        if (m22 < 0) {
            if (m00 >m11) {
                float t = 1 + m00 -m11 -m22;
                float q = quat( t, m01+m10, m20+m02, m12-m21 );
            }
            else {
                float t = 1 -m00 + m11 -m22;
                float q = quat( m01+m10, t, m12+m21, m20-m02 );
            }
        }
        else {
            if (m00 < -m11) {
                float t = 1 -m00 -m11 + m22;
                float q = quat( m20+m02, m12+m21, t, m01-m10 );
            }
            else {
                float t = 1 + m00 + m11 + m22;
                float q = quat( m12-m21, m20-m02, m01-m10, t );
            }
        }
        q *= 0.5 / sqrt(t);
        float q1 = q[0][0];
        float q2 = q[0][1];
        float q3 = q[0][2];
        float q4 = q[0][3];


        geometry_msgs::Pose start_pose2;
        start_pose2.orientation.x = q1;
        start_pose2.orientation.y = q2;
        start_pose2.orientation.z = q3;
        start_pose2.orientation.w = q4;
        start_pose2.position.x = -y;
        start_pose2.position.y = z;
        start_pose2.position.z = -x;
        state->setFromIK(joint_model_group, start_pose2);
        std::vector<double> joint_values;

        state->copyJointGroupPositions(joint_model_group, joint_values);
        

        std::cout << "Joint value:";
        for (auto j : joint_values)
          std::cout << j << ",";
        std::cout << std::endl;

        sensor_msgs::JointState joint_state;
        joint_state.position.resize(6);
        joint_state.name.resize(6);
        joint_state.name[0] = "shoulder_pan_joint";
        joint_state.name[1] = "shoulder_lift_joint";
        joint_state.name[2] = "elbow_joint";
        joint_state.name[3] = "wrist_1_joint";
        joint_state.name[4] = "wrist_2_joint";
        joint_state.name[5] = "wrist_3_joint";

        std::vector<double>
            joint_group_positions;
        for (int i = 0; i < 6; i++)
        {
          joint_group_positions.push_back(0.0);
        }
        state->setVariablePositions(joint_state.name, joint_group_positions);
        // group->setStartState(state(model->getModel()));
        std::cout << "Joint names:";
        for (auto n : joint_model_group->getJointModelNames())
          std::cout << n << ",";
        std::cout << std::endl;

        // joint_group_positions = {0, -M_PI / 2, 0.0, 0.0, -1.5708, 1.06041};
        group->setJointValueTarget(joint_values);
          }

   
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sampleforrange");

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
  
  
  setposegoal(&move_group_interface, &planning_scene_interface, &start_state, &robot_model_loader);



  // execute
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("ur5", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // visualise the paln path

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  ROS_INFO_NAMED("ur5", "Visualizing plan 1 as trajectory line");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // execute
  moveit_msgs::RobotTrajectory trajectory;
  move_group_interface.execute(trajectory);

  return 0;
}
