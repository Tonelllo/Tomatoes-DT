#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "image_geometry/pinhole_camera_model.h"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/console.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <boost/bind/bind.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <cctype>
#include <cmath>
#include <map>
#include <ros/ros.h>
#include <strings.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/transform_datatypes.h"
#include "tomato_vision_manager.h"
#include <pcl_conversions/pcl_conversions.h>

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
  m_point_sub_.subscribe(m_nh_, "/xtion/depth_registered/image_raw", 1);
  m_sync_ = std::make_shared<message_filters::Synchronizer<Sync_policy_>>(10);
  m_sync_->connectInput(m_pose_sub_, m_camera_sub_, m_point_sub_);
  m_sync_->registerCallback(&VisionManager::computeDistances, this);

  m_tomato_position_publisher_ = m_nh_.advertise<geometry_msgs::PoseArray>("/tomato_vision_manager/tomato_position", 1);
}

void VisionManager::createHeadClient()
{
  ROS_INFO("Creating action client to head controller ...");

  m_head_client_.reset(new head_control_client("/head_controller/follow_joint_trajectory"));

  ROS_INFO("Client created");

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

Eigen::Matrix4f VisionManager::stampedTransform2Matrix4f(const tf::StampedTransform& in)
{
  Eigen::Matrix4f ret;

  tf::Vector3 translation;
  ret(0, 3) = translation.x();
  ret(1, 3) = translation.y();
  ret(2, 3) = translation.z();
  ret(3, 3) = 1;

  tf::Matrix3x3 rotation = in.getBasis();
  for (size_t y = 0; y < 3; y++)
  {
    for (size_t x = 0; x < 3; x++)
    {
      ret(y, x) = rotation[y][x];
    }
  }

  return ret;
}

// TODO check for ::constPtr
void VisionManager::computeDistances(geometry_msgs::PoseArray msg, sensor_msgs::CameraInfo info,
                                     sensor_msgs::Image depthInfo)
{
  /**
   *                    X
   *      *--------------->
   *      |
   *      |
   *    Y |
   *      V
   */

  geometry_msgs::PoseArray positions;
  cv::Mat f32image;
  cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(depthInfo, sensor_msgs::image_encodings::TYPE_32FC1);
  cvPtr->image.copyTo(f32image);
  image_geometry::PinholeCameraModel pinholeModel;
  pinholeModel.fromCameraInfo(info);
  for (geometry_msgs::Pose pose : msg.poses)
  {
    // ROS_INFO("%f %f %f", pose.position.x, pose.position.y, pose.position.z);
    // TODO It's created each iteration
    // Z is the class and y is the id
    std::map<std::string, int> vals;
    geometry_msgs::Pose position;
    int x = round(pose.orientation.x);
    int y = round(pose.orientation.y);
    cv::Point3d ray = pinholeModel.projectPixelTo3dRay(cv::Point2d(y,x)); // TODO check order
    position.orientation.w = pose.orientation.z; // Assign the class

    if (depthInfo.encoding != "32FC1")
    {
      ROS_ERROR("Wrong image encoding for depth data");
    }

    // cv::circle(f32image, cv::Point(x, y), 15, cv::Scalar(0, 0, 0), 3);
    // cv::circle(f32image, cv::Point(x, y), 5, cv::Scalar(255, 255, 255), 3);
    float depth = f32image.at<float>(y, x);
    ray *= depth;
    // TODO there are strange things going on with the order
    position.orientation.x = ray.y;
    position.orientation.y = -ray.x;
    // TODO NOTE CHECK
    position.orientation.z = ray.z;
    positions.poses.push_back(position);
  }
  // cv::imshow("cane", f32image);
  // cv::waitKey(100);

  m_tomato_position_publisher_.publish(positions);
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
