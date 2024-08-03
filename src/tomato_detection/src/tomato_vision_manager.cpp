#include "ros/console.h"
#include "ros/init.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include <boost/bind/bind.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <cstdint>
#include <map>
#include <ros/ros.h>
#include <strings.h>
#include "tomato_vision_manager.h"

static double deg2rad(double degrees)
{
  return degrees * (M_PI / 180);
}

VisionManager::VisionManager(ros::NodeHandle& nh)
{
  m_head_goal_.trajectory.joint_names.push_back("head_1_joint");
  m_head_goal_.trajectory.joint_names.push_back("head_2_joint");

  m_nh_ = nh;
  m_best_pos_client_ = m_nh_.serviceClient<tomato_detection::BestPos>("tomato_counting/get_best_tilt");

  m_pose_sub_.subscribe(m_nh_, "/tomato_detection/detected_tomatoes", 1);
  m_camera_sub_.subscribe(m_nh_, "/xtion/depth_registered/camera_info", 1);
  m_point_sub_.subscribe(m_nh_, "/xtion/depth_registered/points", 1);
  m_sync_ = std::make_shared<message_filters::Synchronizer<Sync_policy_>>(10);
  m_sync_->connectInput(m_pose_sub_, m_camera_sub_, m_point_sub_);
  m_sync_->registerCallback(&VisionManager::computeDistances, this);
}

void VisionManager::createHeadClient()
{
  ROS_INFO("Creating action client to head controller ...");

  m_head_client_.reset(new head_control_client("/head_controller/follow_joint_trajectory"));

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while (!m_head_client_->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
  {
    ROS_DEBUG("Waiting for the head_controller_action server to come up");
    ++iterations;
  }

  if (iterations == max_iterations)
    throw std::runtime_error(
        "Error in createHeadClient: head controller "
        "action server not available");
}

void VisionManager::lookUp()
{
  m_head_client_->cancelAllGoals();  // TODO not a good practice
  // Set the number of points in trajectory
  m_head_goal_.trajectory.points.resize(1);

  int index = 0;
  m_head_goal_.trajectory.points[index].positions.resize(2);
  m_head_goal_.trajectory.points[index].positions[0] = 0.0;
  m_head_goal_.trajectory.points[index].positions[1] = deg2rad(20);

  // Set the number of velocities in the array
  m_head_goal_.trajectory.points[index].velocities.resize(2);

  // This sets the velocity at which the head will pass
  // through the waypoint
  m_head_goal_.trajectory.points[index].velocities[0] = 0.0;
  m_head_goal_.trajectory.points[index].velocities[1] = 0.0;

  m_head_goal_.trajectory.points[index].time_from_start = ros::Duration(5.0);
  m_head_client_->sendGoal(m_head_goal_);
  ros::Duration(1).sleep();
}

void VisionManager::scan()
{
  m_head_client_->cancelAllGoals();  // TODO not a good practice
  m_head_goal_.trajectory.points.resize(1);

  int index = 0;
  m_head_goal_.trajectory.points[index].positions.resize(2);
  m_head_goal_.trajectory.points[index].positions[0] = 0.0;
  m_head_goal_.trajectory.points[index].positions[1] = deg2rad(-30.0);

  // Set the number of velocities in the array
  m_head_goal_.trajectory.points[index].velocities.resize(2);
  // This sets the velocity at which the head will pass
  // through the waypoint
  m_head_goal_.trajectory.points[index].velocities[0] = 0.0;
  m_head_goal_.trajectory.points[index].velocities[1] = 0.0;

  m_head_goal_.trajectory.points[index].time_from_start = ros::Duration(10.0);
  m_head_client_->sendGoal(m_head_goal_);
  ros::Duration(1).sleep();
}

void VisionManager::lookAtBestPosition()
{
  m_head_client_->cancelAllGoals();  // TODO not a good practice
  m_head_goal_.trajectory.points.resize(1);

  int index = 0;
  m_head_goal_.trajectory.points[index].positions.resize(2);
  m_head_goal_.trajectory.points[index].positions[0] = 0.0;
  m_head_goal_.trajectory.points[index].positions[1] = m_best_position_;

  // Set the number of velocities in the array
  m_head_goal_.trajectory.points[index].velocities.resize(2);
  // This sets the velocity at which the head will pass
  // through the waypoint
  m_head_goal_.trajectory.points[index].velocities[0] = 0.0;
  m_head_goal_.trajectory.points[index].velocities[1] = 0.0;

  m_head_goal_.trajectory.points[index].time_from_start = ros::Duration(5.0);
  m_head_client_->sendGoal(m_head_goal_);
  ros::Duration(1).sleep();
}

bool VisionManager::isGoalReached()
{
  return m_head_client_->getState().isDone();
}

void VisionManager::startYOLOScan()
{
  m_best_pos_msg_.request.activate = true;
  m_best_pos_client_.call(m_best_pos_msg_);
}

// TODO check for ::constPtr
void VisionManager::computeDistances(geometry_msgs::PoseArray msg, sensor_msgs::CameraInfo info,
                                     sensor_msgs::PointCloud2 pc)
{
  ROS_INFO("VVVVVVVVVVVVVVVVVVVVVVVV");
  for (geometry_msgs::Pose pose : msg.poses)
  {
    // ROS_INFO("%f %f %f", pose.position.x, pose.position.y, pose.position.z);
    // TODO It's created each iteration
    std::map<std::string, int> vals;
    int x = round(pose.position.x);
    int y = round(pose.position.y);


    for (sensor_msgs::PointField pf : pc.fields)
    {
      // ROS_INFO("%s, %d, %d, %d", pf.name.c_str(), pf.offset, pf.datatype, pf.count);
      vals.insert({ pf.name.c_str(), pf.offset });
    }

    uint32_t start = pc.point_step * x + y * pc.row_step;
    // for(size_t index = start; index < start + pc.point_step; index++){
    //   ROS_INFO("%d", pc.data[index]);
    // }

    ROS_INFO("x: %d, y: %d, z: %d", pc.data[start + vals["x"]], pc.data[start + vals["y"]], pc.data[start + vals["z"]]);
  }
  ROS_INFO("########################");
}

void VisionManager::getBestPosition()
{
  m_best_pos_msg_.request.activate = false;
  m_best_pos_client_.call(m_best_pos_msg_);

  if (m_best_pos_msg_.response.is_valid)
  {
    m_best_position_ = m_best_pos_msg_.response.bestpos;
  }
  else
  {
    ROS_ERROR("INVALID RESPONSE");
    exit(1);
  }

  ROS_INFO("Best position: %f", m_best_position_);
}
