#include <boost/iterator/minimum_category.hpp>
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_spline_parameterization.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "tomato_trajectory_splicer/SpliceService.h"
#include "moveit/robot_model_loader/robot_model_loader.h"

class TrajectorySplicer
{
  ros::NodeHandle mNh_;
  ros::ServiceServer mTimeParametrizerSer_;
  ros::Subscriber mPointSub_;

public:
  bool serviceCallback(tomato_trajectory_splicer::SpliceServiceRequest& req,
                       tomato_trajectory_splicer::SpliceServiceResponse& res)
  {
    robot_model_loader::RobotModelLoader RobotModelLoader("robot_description");
    moveit::core::RobotModelPtr RobotModel = RobotModelLoader.getModel();
    moveit::core::RobotStatePtr ReferenceState(new moveit::core::RobotState(RobotModel));
    ReferenceState->setToDefaultValues();

    moveit_msgs::RobotTrajectory rt1 = req.traj1;
    moveit_msgs::RobotTrajectory rt2 = req.traj2;
    moveit_msgs::RobotTrajectory spliced(rt1);
    auto last = rt1.joint_trajectory.points.back().time_from_start;
    spliced.joint_trajectory.points.pop_back();

    /*bool first = true;*/
    for (const auto& elem : rt2.joint_trajectory.points)
    {
      /*if (first)*/
      /*{*/
      /*  first = false;*/
      /*  continue;*/
      /*}*/
      trajectory_msgs::JointTrajectoryPoint new_point = elem;
      new_point.time_from_start += ros::Duration(last);
      spliced.joint_trajectory.points.push_back(new_point);
    }

    trajectory_processing::IterativeSplineParameterization time_param;

    robot_trajectory::RobotTrajectory robotTraj(RobotModel, "arm_torso");
    robotTraj.setRobotTrajectoryMsg(*ReferenceState, spliced);
    bool success = time_param.computeTimeStamps(robotTraj);
    if (!success)
    {
      ROS_WARN("Not able to splice trajectories");
      res.success = false;
      return true;
    }
    moveit_msgs::RobotTrajectory out;
    robotTraj.getRobotTrajectoryMsg(out);
    for (auto& point : out.joint_trajectory.points)
    {
      point.time_from_start += ros::Duration(0, 100);
    }
    res.success = true;
    res.res = out;
    return true;
  }

  TrajectorySplicer(ros::NodeHandle& nh)
  {
    mNh_ = nh;
    mTimeParametrizerSer_ =
        mNh_.advertiseService("/tomato_sync/getSplicedTraj", &TrajectorySplicer::serviceCallback, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_publisher");
  ros::NodeHandle nh;

  TrajectorySplicer ou(nh);

  std::cout << "############### SPLICER STARTED ###############\n";

  ros::spin();
}
