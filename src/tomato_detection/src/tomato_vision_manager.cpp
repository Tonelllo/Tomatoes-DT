#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/service.h"
#include "ros/service_client.h"
#include "tomato_detection/BestPosRequest.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <strings.h>
#include <tomato_detection/BestPos.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    head_control_client;
typedef boost::shared_ptr<head_control_client> head_control_client_Ptr;

static double deg2rad(double degrees) { return degrees * (M_PI / 180); }

void createHeadClient(head_control_client_Ptr &actionClient) {
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset(
      new head_control_client("/head_controller/follow_joint_trajectory"));

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() &&
         iterations < max_iterations) {
    ROS_DEBUG("Waiting for the head_controller_action server to come up");
    ++iterations;
  }

  if (iterations == max_iterations)
    throw std::runtime_error("Error in createHeadClient: head controller "
                             "action server not available");
}

void look_up(control_msgs::FollowJointTrajectoryGoal &goal) {
  // Set the number of points in trajectory
  goal.trajectory.points.resize(1);

  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = deg2rad(20);

  // Set the number of velocities in the array
  goal.trajectory.points[index].velocities.resize(2);

  // This sets the velocity at which the head will pass
  // through the waypoint
  goal.trajectory.points[index].velocities[0] = 0.0;
  goal.trajectory.points[index].velocities[1] = 0.0;

  goal.trajectory.points[index].time_from_start = ros::Duration(5.0);
}

void scan(control_msgs::FollowJointTrajectoryGoal &goal) {
  goal.trajectory.points.resize(1);

  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = deg2rad(-30.0);

  // Set the number of velocities in the array
  goal.trajectory.points[index].velocities.resize(2);
  // This sets the velocity at which the head will pass
  // through the waypoint
  goal.trajectory.points[index].velocities[0] = 0.0;
  goal.trajectory.points[index].velocities[1] = 0.0;

  goal.trajectory.points[index].time_from_start = ros::Duration(15.0);
}

void lookAtBestPosition(control_msgs::FollowJointTrajectoryGoal &goal,
                        float bestPosition) {
  goal.trajectory.points.resize(1);

  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = bestPosition;

  // Set the number of velocities in the array
  goal.trajectory.points[index].velocities.resize(2);
  // This sets the velocity at which the head will pass
  // through the waypoint
  goal.trajectory.points[index].velocities[0] = 0.0;
  goal.trajectory.points[index].velocities[1] = 0.0;

  goal.trajectory.points[index].time_from_start = ros::Duration(20.0);
}

void waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal &goal) {
  goal.trajectory.joint_names.push_back("head_1_joint");
  goal.trajectory.joint_names.push_back("head_2_joint");

  // Set the number of points in trajectory
  goal.trajectory.points.resize(3);

  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = deg2rad(20);

  // Set the number of velocities in the array
  goal.trajectory.points[index].velocities.resize(2);

  // This sets the velocity at which the head will pass
  // through the waypoint
  goal.trajectory.points[index].velocities[0] = 0.0;
  goal.trajectory.points[index].velocities[1] = 0.0;

  goal.trajectory.points[index].time_from_start = ros::Duration(5.0);

  index++;

  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = deg2rad(-30.0);

  // Set the number of velocities in the array
  goal.trajectory.points[index].velocities.resize(2);
  // This sets the velocity at which the head will pass
  // through the waypoint
  goal.trajectory.points[index].velocities[0] = 0.0;
  goal.trajectory.points[index].velocities[1] = 0.0;

  goal.trajectory.points[index].time_from_start = ros::Duration(15.0);

  index++;

  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = deg2rad(0.0);

  // Set the number of velocities in the array
  goal.trajectory.points[index].velocities.resize(2);
  // This sets the velocity at which the head will pass
  // through the waypoint
  goal.trajectory.points[index].velocities[0] = 0.0;
  goal.trajectory.points[index].velocities[1] = 0.0;

  goal.trajectory.points[index].time_from_start = ros::Duration(20.0);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "vision_manager");

  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(
          10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  ros::service::waitForService("tomato_counting/get_best_tilt");
  ros::ServiceClient bestPosClient =
      nh.serviceClient<tomato_detection::BestPos>(
          "tomato_counting/get_best_tilt");

  head_control_client_Ptr HeadClient;
  createHeadClient(HeadClient);
  HeadClient->cancelAllGoals();
  
  control_msgs::FollowJointTrajectoryGoal head_goal;
  head_goal.trajectory.joint_names.push_back("head_1_joint");
  head_goal.trajectory.joint_names.push_back("head_2_joint");
  
  look_up(head_goal);
  head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  HeadClient->sendGoal(head_goal);

  while (!(HeadClient->getState().isDone()) && ros::ok()) {
    ros::Duration(1).sleep(); // sleep for four seconds
  }

  tomato_detection::BestPos bp;
  bp.request.activate = true;
  bestPosClient.call(bp);

  HeadClient->cancelAllGoals();
  scan(head_goal);
  HeadClient->sendGoal(head_goal);
  head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  while (!(HeadClient->getState().isDone()) && ros::ok()) {
    ros::Duration(1).sleep(); // sleep for four seconds
  }

  float bestPosition = 0;

  bp.request.activate = false;
  bestPosClient.call(bp);
  if (bp.response.is_valid) {
    bestPosition = bp.response.bestpos;
  } else {
    ROS_ERROR("INVALID RESPONSE");
    exit(1);
  }

  ROS_INFO("Best position: %f", bestPosition);
  HeadClient->cancelAllGoals();
  lookAtBestPosition(head_goal, bestPosition);
  HeadClient->sendGoal(head_goal);
  head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  while (!(HeadClient->getState().isDone()) && ros::ok()) {
    ros::Duration(1).sleep(); // sleep for four seconds
  }
  // waypoints_head_goal(head_goal);
  //
  // // The action will start 1 second from now
  // // head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  //  HeadClient->sendGoal(head_goal);
  //  while (!(HeadClient->getState().isDone()) && ros::ok()) {
  //    ros::Duration(4).sleep(); // sleep for four seconds
  //  }

  return 0;
}
