#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseStamped.h"
#include <chrono>
#include <mutex>
#include <thread>

std::mutex m;

void planWorker(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::PoseStamped& target_pose,
                moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  m.lock();
  move_group.setPoseTarget(target_pose);
  std::cout << target_pose.pose.position.x << std::endl;
  move_group.plan(plan);
  m.unlock();
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialize MoveIt Commander
  moveit::planning_interface::MoveGroupInterface move_group("arm_torso");

  // Set the target pose
  geometry_msgs::PoseStamped target_pose_1;
  target_pose_1.header.frame_id = "base_footprint";
  target_pose_1.pose.position.x = 0.8;
  target_pose_1.pose.position.y = 0;
  target_pose_1.pose.position.z = 0.9;
  target_pose_1.pose.orientation.x = 0.7;
  target_pose_1.pose.orientation.y = 0.0;
  target_pose_1.pose.orientation.z = 0.0;
  target_pose_1.pose.orientation.w = 0.7;

  geometry_msgs::PoseStamped target_pose_2;
  target_pose_2.header.frame_id = "base_footprint";
  target_pose_2.pose.position.x = 0.7;
  target_pose_2.pose.position.y = 0;
  target_pose_2.pose.position.z = 0.9;
  target_pose_2.pose.orientation.x = 0.7;
  target_pose_2.pose.orientation.y = 0.0;
  target_pose_2.pose.orientation.z = 0.0;
  target_pose_2.pose.orientation.w = 0.7;

  // Plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_1;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_2;

  std::thread th1(planWorker, std::ref(move_group), std::ref(target_pose_1), std::ref(my_plan_1));
  std::thread th2(planWorker, std::ref(move_group), std::ref(target_pose_2), std::ref(my_plan_2));

  th1.join();
  th2.join();

  std::cout << my_plan_1.planning_time_ << std::endl;
  std::cout << my_plan_2.planning_time_ << std::endl;

  move_group.execute(my_plan_1);
  move_group.stop();
  move_group.clearPoseTargets();

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(30s);

  move_group.execute(my_plan_2);
  move_group.stop();
  move_group.clearPoseTargets();

  ros::shutdown();
  return 0;
}
