#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "image_geometry/pinhole_camera_model.h"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "ros/console.h"
#include "ros/duration.h"
#include "ros/time.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <boost/bind/bind.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <cctype>
#include <cmath>
#include <functional>
#include <map>
#include <Eigen/Core>
#include <ros/ros.h>
#include <strings.h>
#include "tomato_vision_manager.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

static double deg2rad(double degrees)
{
  return degrees * (M_PI / 180.0);
}

static double rad2deg(double rad)
{
  return rad * (180.0 / M_PI);
}

VisionManager::VisionManager(ros::NodeHandle& nh)
{
  m_head_goal_.trajectory.joint_names.push_back("head_1_joint");
  m_head_goal_.trajectory.joint_names.push_back("head_2_joint");

  m_nh_ = nh;
  m_best_pos_client_ = m_nh_.serviceClient<tomato_detection::BestPos>("tomato_counting/get_best_tilt");

  m_pose_sub_ = m_nh_.subscribe<geometry_msgs::PoseArray>("/tomato_detection/detected_tomatoes", 5, &VisionManager::computeDistances, this);

  m_camera_sub_.subscribe(m_nh_, "/tomato_sync/image_params", 5);
  m_point_sub_.subscribe(m_nh_, "/tomato_sync/image_depth", 5);

  m_point_cache_.setCacheSize(70);
  m_point_cache_.connectInput(m_point_sub_);
  m_camera_cache_.setCacheSize(70);
  m_camera_cache_.connectInput(m_camera_sub_);

  m_tomato_position_publisher_ = m_nh_.advertise<geometry_msgs::PoseArray>("/tomato_vision_manager/tomato_position", 1);
  m_tomato_position_server_ = m_nh_.advertiseService("/tomato_vision_manager/tomato_position_service",
                                                     &VisionManager::getLatestTomatoPositions, this);
  m_goal_reached_ = false;
  m_tfListener_ = new tf2_ros::TransformListener(m_buffer_);
}

bool VisionManager::getLatestTomatoPositions(tomato_detection::LatestTomatoPositionsRequest& req,
                                             tomato_detection::LatestTomatoPositionsResponse&res)
{
  poseMutex.lock();
  res.tomatoes = m_latest_positions;
  poseMutex.unlock();
  return true;
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

void VisionManager::goalReachedCallback()
{
  m_goal_reached_ = true;
}

bool VisionManager::isGoalReached()
{
  if (m_goal_reached_)
  {
    m_goal_reached_ = false;
    return true;
  }
  return false;
}

void VisionManager::resetTrajectory()
{
  m_head_client_->cancelAllGoals();
  m_head_goal_.trajectory.points.clear();
  ros::Duration(1).sleep();
}

void VisionManager::setNextPoint(float head_tilt, float time_to_reach)
{
  m_head_goal_.trajectory.points.resize(1);

  m_head_goal_.trajectory.points[0].positions.resize(2);
  m_head_goal_.trajectory.points[0].positions[0] = 0.0;
  m_head_goal_.trajectory.points[0].positions[1] = deg2rad(head_tilt);

  // Set the number of velocities in the array
  m_head_goal_.trajectory.points[0].velocities.resize(2);

  // This sets the velocity at which the head will pass
  // through the waypoint
  m_head_goal_.trajectory.points[0].velocities[0] = 0.0;
  m_head_goal_.trajectory.points[0].velocities[1] = 0.0;

  m_head_goal_.trajectory.points[0].time_from_start = ros::Duration(time_to_reach);
  m_head_client_->sendGoal(m_head_goal_, std::bind(&VisionManager::goalReachedCallback, this));
}

void VisionManager::lookUp()
{
  setNextPoint(20, 3);
}

void VisionManager::scan()
{
  setNextPoint(-30, 5);
}

void VisionManager::lookAtBestPosition()
{
  setNextPoint(rad2deg(m_best_position_), 3);
}

void VisionManager::startYOLOScan()
{
  m_best_pos_msg_.request.activate = true;
  m_best_pos_client_.call(m_best_pos_msg_);
}

tf2::Transform VisionManager::stampedTransform2tf2Transform(geometry_msgs::TransformStamped in)
{
  tf2::Quaternion q(in.transform.rotation.x, in.transform.rotation.y, in.transform.rotation.z, in.transform.rotation.w);
  tf2::Vector3 t(in.transform.translation.x, in.transform.translation.y, in.transform.translation.z);

  return tf2::Transform(q, t);
}

// TODO Get correct transform
void VisionManager::computeDistances(geometry_msgs::PoseArray msg)
{
  /**
   *                    X
   *      *--------------->
   *      |
   *      |
   *    Y |
   *      V
   */
  sensor_msgs::Image depthInfo;
  sensor_msgs::CameraInfo info;
  auto auxInfo = m_camera_cache_.getInterval(msg.header.stamp, msg.header.stamp);
  auto auxDepthInfo = m_point_cache_.getInterval(msg.header.stamp, msg.header.stamp);

  if(auxInfo.empty() || auxDepthInfo.empty())
    return;

  info = *auxInfo[0];
  depthInfo = *auxDepthInfo[0];

  geometry_msgs::PoseArray positions;
  cv::Mat f32image;
  cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(depthInfo, sensor_msgs::image_encodings::TYPE_32FC1);
  cvPtr->image.copyTo(f32image);
  m_camera_model_.fromCameraInfo(info);

  geometry_msgs::TransformStamped transformStamped;
  tf2::Transform camera_to_torso;
  try
  {
    transformStamped = m_buffer_.lookupTransform("base_footprint", m_camera_model_.tfFrame(), ros::Time(0));
    camera_to_torso = stampedTransform2tf2Transform(transformStamped);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  bool first = true;
  for (geometry_msgs::Pose pose : msg.poses)
  {
    // ROS_INFO("%f %f %f", pose.position.x, pose.position.y, pose.position.z);
    // TODO It's created each iteration
    // Z is the class and y is the id
    std::map<std::string, int> vals;
    geometry_msgs::Pose position;
    int x = round(pose.orientation.x);
    int y = round(pose.orientation.y);
    cv::Point3d ray = m_camera_model_.projectPixelTo3dRay(cv::Point2d(x, y));
    position.orientation.x = pose.orientation.z;  // Assign the class
    position.orientation.y = pose.orientation.w;  // Assign the id

    if (depthInfo.encoding != "32FC1")
    {
      ROS_ERROR("Wrong image encoding for depth data");
    }

    cv::circle(f32image, cv::Point(x, y), 15, cv::Scalar(0, 0, 0), 3);
    cv::circle(f32image, cv::Point(x, y), 5, cv::Scalar(255, 255, 255), 3);
    float depth = f32image.at<float>(y, x);  // NOTE Row, Col

    // Diameter
    position.orientation.z = (pose.position.x * depth) / m_camera_model_.fx();

    ray *= depth + position.orientation.z / 2;
    tf2::Vector3 cameraPoint(ray.x, ray.y, ray.z);
    tf2::Vector3 bodyFixedPoint;

    bodyFixedPoint = camera_to_torso * cameraPoint;

    position.position.x = bodyFixedPoint.x();
    position.position.y = bodyFixedPoint.y();
    position.position.z = bodyFixedPoint.z();
    positions.poses.push_back(position);
  }
  cv::imshow("cane", f32image);
  cv::waitKey(100);
  poseMutex.lock();
  m_latest_positions = positions;
  poseMutex.unlock();
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
