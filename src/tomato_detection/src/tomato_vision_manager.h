#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "image_geometry/pinhole_camera_model.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseArray.h"
#include "ros/publisher.h"
#include <tomato_detection/BestPos.h>
#include <tomato_detection/LatestTomatoPositions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <memory>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "Eigen/Dense"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Transform.h>
#include "tf2_ros/transform_listener.h"
#include "message_filters/cache.h"

class VisionManager
{
  using head_control_client = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
  using head_control_client_ptr = boost::shared_ptr<head_control_client>;
  head_control_client_ptr m_head_client_;
  image_geometry::PinholeCameraModel m_camera_;
  control_msgs::FollowJointTrajectoryGoal m_head_goal_;
  float m_best_position_;
  tomato_detection::BestPos m_best_pos_msg_;
  ros::NodeHandle m_nh_;
  ros::ServiceClient m_best_pos_client_;
  image_geometry::PinholeCameraModel m_camera_model_;
  tf2::Transform stampedTransform2tf2Transform(geometry_msgs::TransformStamped);

  using Sync_policy_ = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray,
                                                                       sensor_msgs::CameraInfo, sensor_msgs::Image>;
  ros::Subscriber m_pose_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> m_camera_sub_;
  // message_filters::Subscriber<sensor_msgs::PointCloud2> m_point_sub_;
  message_filters::Subscriber<sensor_msgs::Image> m_point_sub_;
  message_filters::Cache<sensor_msgs::Image>pointCache;
  message_filters::Cache<sensor_msgs::CameraInfo>infoCache;
  std::shared_ptr<message_filters::Synchronizer<Sync_policy_>> m_sync_;

  void setNextPoint(float, float);
  void goalReachedCallback();
  ros::Publisher m_tomato_position_publisher_;
  ros::ServiceServer m_tomato_position_server_;
  bool m_goal_reached_;
  tf2_ros::Buffer m_buffer_;
  tf2_ros::TransformListener* m_tfListener_;
  std::mutex poseMutex;
  geometry_msgs::PoseArray m_latest_positions;

public:
  VisionManager(ros::NodeHandle&);
  void resetTrajectory();
  bool isGoalReached();
  void createHeadClient();
  void lookUp();
  void scan();
  void lookAtBestPosition();
  void computeDistances(geometry_msgs::PoseArray);
  void getBestPosition();
  void startYOLOScan();
  bool getLatestTomatoPositions(tomato_detection::LatestTomatoPositionsRequest& req,
                                tomato_detection::LatestTomatoPositionsResponse& res);
};
