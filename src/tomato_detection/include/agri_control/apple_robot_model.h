#ifndef APPLE_ROBOT_MODEL_H
#define APPLE_ROBOT_MODEL_H

#include <ros/ros.h>
#include <rml/RML.h>

namespace rml {

class AppleRobotModel : public ArmModel
{

public:
    AppleRobotModel(std::shared_ptr<ros::NodeHandle> nodeHandle, const std::string id);
    virtual ~AppleRobotModel();

    std::vector<double> jointOffsetsSim;
    std::vector<double> jointDirectionSwapSim;

    std::shared_ptr<ros::NodeHandle> nh_;
};

}

#endif /* __APPLE_ROBOT_MODEL_H__ */