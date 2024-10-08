#include <ros/ros.h>
#include "agri_control/kinematic_controller.hpp"
#include <agri_control/apple_robot_model.h>

int main(int argc, char ** argv)
{
    bool simulatedDriver = true; // static_cast<bool>(std::atoi(argv[1])); // TODO check

    std::cout << "simulatedDriver: " << simulatedDriver << std::endl;

    std::string nodeName = "kinematic_control_node";
    ros::init(argc, argv, nodeName);

    // Name of conf file
    std::string filename = "kcl_applerobot.conf"; 

    ROS_INFO_STREAM("Starting node: " << nodeName);

    std::shared_ptr<ros::NodeHandle> nh_p = std::make_shared<ros::NodeHandle>("~");
    //KinematicController kinematicControllerLeft(nh_p, filename, simulatedDriver, "/left_");
    KinematicController kinematicControllerRight(nh_p, filename, simulatedDriver, "/right_");
    ros::spin();

    return 0;
}
