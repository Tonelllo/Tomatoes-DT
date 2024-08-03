#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "image_geometry/pinhole_camera_model.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseArray.h"
#include "ros/forwards.h"
#include <tomato_detection/BestPos.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <sensor_msgs/PointCloud2.h>

class VisionManager
{
  using head_control_client =  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
  using head_control_client_ptr = boost::shared_ptr<head_control_client>;
  head_control_client_ptr m_head_client_;
  image_geometry::PinholeCameraModel m_camera_;
  control_msgs::FollowJointTrajectoryGoal m_head_goal_;
  float m_best_position_;
  tomato_detection::BestPos m_best_pos_msg_;
  ros::NodeHandle m_nh_;
  ros::ServiceClient m_best_pos_client_;

  using Sync_policy_ = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2>;
  message_filters::Subscriber<geometry_msgs::PoseArray> m_pose_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> m_camera_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> m_point_sub_;
  
  std::shared_ptr<message_filters::Synchronizer<Sync_policy_>> m_sync_;

public:
  VisionManager(ros::NodeHandle&);
  void createHeadClient();
  void lookUp();
  void scan();
  void lookAtBestPosition();
  void computeDistances(geometry_msgs::PoseArray, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);
  void getBestPosition();
  void startYOLOScan();
  bool isGoalReached();
};
