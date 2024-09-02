#include <ros/ros.h>
#include <boost/variant/detail/visitation_impl.hpp>
#include "ros/init.h"
#include "ros/service.h"
#include "tomato_vision_manager.h"

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

  vm.createHeadClient();

  ros::Rate rate(10);
  enum class states
  {
    LOOK_UP,
    SCAN,
    GOTO_BEST,
    COMPUTE_DISTANCES
  };
  states state = states::COMPUTE_DISTANCES;
  bool once = true;
  while (ros::ok())
  {
    switch (state)
    {
      case states::LOOK_UP:
        if (once)
        {
          ROS_INFO("Looking up");

          vm.lookUp();
          once = false;
        }

        if (vm.isGoalReached())
        {
          state = states::SCAN;
          once = true;
        }
        break;
      case states::SCAN:
        if (once)
        {
          ROS_INFO("Scanning");

          vm.startYOLOScan();
          vm.scan();

          once = false;
        }

        if (vm.isGoalReached())
        {
          state = states::GOTO_BEST;
          once = true;
        }
        break;

      case states::GOTO_BEST:
        if (once)
        {
          ROS_INFO("Going to best position");
          vm.getBestPosition();
          vm.lookAtBestPosition();
          once = false;
        }

        if (vm.isGoalReached())
        {
          state = states::COMPUTE_DISTANCES;
          once = true;
        }
        break;
      case states::COMPUTE_DISTANCES:
        if (once)
        {
          // ROS_INFO("Compute distances");
          once = true;
        }
        break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
