#include <ros/ros.h>
#include "ros/node_handle.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
