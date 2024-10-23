#include <ros/ros.h>
#include <atomic>
#include <boost/variant/detail/visitation_impl.hpp>
#include "ros/init.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include "tomato_detection/CurrentVisionStateResponse.h"
#include "tomato_vision_manager.h"
#include "std_srvs/Empty.h"
#include "tomato_detection/CurrentVisionState.h"

namespace
{
enum class States
{
  IDLE,
  LOOK_UP,
  SCAN,
  GOTO_BEST,
  COMPUTE_DISTANCES
};

std::atomic<States> state(States::IDLE);

bool stateMachineRestarter(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  state = States::LOOK_UP;
  return true;
}

bool stateProvider(tomato_detection::CurrentVisionStateRequest& req, tomato_detection::CurrentVisionStateResponse& res)
{
  if (state == States::COMPUTE_DISTANCES)
  {
    res.scanFinished = true;
  }
  else
  {
    res.scanFinished = false;
  }
  return true;
}
}  // namespace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vision_manager");
  ros::NodeHandle nh;
  VisionManager vm(nh);

  if (!ros::Time::waitForValid(ros::WallDuration(10.0)))  // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  ros::service::waitForService("tomato_counting/get_best_tilt");
  ros::ServiceServer serv = nh.advertiseService("/tomato_vision_manager/start_scan", stateMachineRestarter);
  ros::ServiceServer stateServer = nh.advertiseService("/tomato_vision_manager/get_state", stateProvider);

  vm.createHeadClient();

  ros::Rate rate(10);
  state = States::LOOK_UP;
  bool once = true;
  vm.resetTrajectory();
  while (ros::ok())
  {
    switch (state)
    {
      case States::IDLE:
        break;
      case States::LOOK_UP:
        if (once)
        {
          ROS_INFO("Looking up");

          vm.lookUp();
          once = false;
        }

        if (vm.isGoalReached())
        {
          state = States::SCAN;
          once = true;
        }
        break;
      case States::SCAN:
        if (once)
        {
          ROS_INFO("Scanning");

          vm.startYOLOScan();
          vm.scan();

          once = false;
        }

        if (vm.isGoalReached())
        {
          state = States::GOTO_BEST;
          once = true;
        }
        break;

      case States::GOTO_BEST:
        if (once)
        {
          ROS_INFO("Going to best position");
          vm.getBestPosition();
          vm.lookAtBestPosition();
          once = false;
        }

        if (vm.isGoalReached())
        {
          state = States::COMPUTE_DISTANCES;
          ROS_INFO("Compute distances");
          once = true;
        }
        break;
      case States::COMPUTE_DISTANCES:
        // Keeps Looping
        break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
