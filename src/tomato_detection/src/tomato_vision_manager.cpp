#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/time.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <boost/bind/bind.hpp>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
#include <ros/ros.h>
#include <strings.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tomato_vision_manager.h"
#include "tf/tf.h"
#include "pcl_ros/transforms.h"
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
  m_point_sub_.subscribe(m_nh_, "/xtion/depth_registered/points", 1);
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
                                     pcl::PointCloud<pcl::PointXYZ> pc)
{
  /**
   *                    X
   *      *--------------->
   *      |
   *      |
   *    Y |
   *      V
   */
  // ROS_INFO("VVVVVVVVVVVVVVVVVVVVVVVV");
  m_camera_model_.fromCameraInfo(info);
  geometry_msgs::PoseArray positions;
  tf::TransformListener tfListener;
  tf::StampedTransform tfStTr;
  sensor_msgs::PointCloud2 camera_frame_pc;
  tfListener.lookupTransform(m_camera_model_.tfFrame(), pc.header.frame_id, ros::Time::now(), tfStTr);
  pcl_ros::transformPointCloud(stampedTransform2Matrix4f(tfStTr), pc, camera_frame_pc);

  for (geometry_msgs::Pose pose : msg.poses)
  {
    // ROS_INFO("%f %f %f", pose.position.x, pose.position.y, pose.position.z);
    // TODO It's created each iteration
    std::map<std::string, int> vals;
    int x = round(pose.orientation.x);
    int y = round(pose.orientation.y);
    geometry_msgs::Pose position;

    for (sensor_msgs::PointField pf : camera_frame_pc.fields)
    {
      // //ROS_INFO("%s, %d, %d, %d", pf.name.c_str(), pf.offset, pf.datatype, pf.count);
      vals.insert({ pf.name.c_str(), pf.offset });
    }

    uint32_t start = camera_frame_pc.width * y + x;

    if (start >= camera_frame_pc.width * camera_frame_pc.height)
    {
      ROS_ERROR("INDEX EXCEEDS POINT CLOUD SIZE");
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(camera_frame_pc, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(camera_frame_pc, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(camera_frame_pc, "z");
    // ROS_INFO("--------------------------------------------");
    // ROS_INFO("--------------------------------------------");

    // ROS_INFO("x: %d, y: %d, z: %d", camera_frame_pc.data[start + vals["x"]], camera_frame_pc.data[start + vals["y"]],
    // camera_frame_pc.data[start + vals["z"]]);

    position.orientation.x = *(iter_x + start);
    position.orientation.y = *(iter_y + start);
    position.orientation.z = *(iter_z + start);
    position.orientation.w = static_cast<int>(pose.orientation.z);
    ROS_INFO("CX: %f, CY: %f, x: %f, y: %f, z: %f, id: %f", pose.orientation.x, pose.orientation.y,
             position.orientation.x, position.orientation.y, position.orientation.z, pose.orientation.w);
    positions.poses.push_back(position);
  }
  cv::Mat outx(cv::Size(640, 480), CV_8UC1);
  cv::Mat outy(cv::Size(640, 480), CV_8UC1);
  cv::Mat outz(cv::Size(640, 480), CV_8UC1);
  int count = 0;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter(camera_frame_pc, "x"); iter != iter.end(); ++iter)
  {
    outx.at<uchar>(count / 640, count % 640) = std::isnan(*iter) ? 0 : 255;
    outy.at<uchar>(count / 640, count % 640) = std::isnan(*(iter+1)) ? 0 : 255;
    outz.at<uchar>(count / 640, count % 640) = std::isnan(*(iter+2)) ? 0 : 255;
    count++;
  }
  cv::imshow("x_coord", outx);
  cv::imshow("y_coord", outy);
  cv::imshow("z_coord", outz);
  cv::waitKey(10);
  m_tomato_position_publisher_.publish(positions);
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
