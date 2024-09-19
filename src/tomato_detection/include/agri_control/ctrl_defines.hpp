#ifndef CTRL_DEFINES_HPP
#define CTRL_DEFINES_HPP

#include <ros/ros.h>
#include "tpik/TPIKlib.h"

#include "tomato_detection/TaskStatus.h"
#include "tomato_detection/ControlData.h"
#include "tomato_detection/DriverCommand.h"

struct RobotInfo {
    std::string arm_state_id;
    std::string gripper_state_id;
    bool requestedEnable;
    bool requestedUnpark;
    std::string toolID;
    Eigen::VectorXd jointsGoal;
    Eigen::TransformationMatrix cartesianGoalFrame;


    bool simulatedDriver;
    Eigen::VectorXd parkedJointsPos;
    Eigen::VectorXd unparkedJointsPos;

    RobotInfo() :
        requestedEnable(false)
    {}

};

struct TasksInfo {

    std::shared_ptr<tpik::Task> task;
    ros::Publisher taskPub;
};

struct KCLConfiguration {

    std::string filename;
    //bool goToHoldAfterMove;
    double loopRate;
    double linearErrorThreshold;
    double angularErrorThreshold;
    double cartesianMoveTimeout;
    double positionGainJoints;
    double positionGainCart;

    Eigen::VectorXd saturationMin, saturationMax;

    KCLConfiguration(std::string conf_filename)
        : filename(conf_filename)
    {
    }

    bool ConfigureFromFile(libconfig::Config& confObj)
    {
        if (!ctb::GetParam(confObj, loopRate, "loopRate"))
            return false;
        if (!ctb::GetParam(confObj, linearErrorThreshold, "linearErrorThreshold"))
            return false;
        if (!ctb::GetParam(confObj, angularErrorThreshold, "angularErrorThreshold"))
            return false;
        if (!ctb::GetParam(confObj, cartesianMoveTimeout, "cartesianMoveTimeout"))
            return false;
        if (!ctb::GetParamVector(confObj, saturationMax, "saturationMax"))
            return false;
        if (!ctb::GetParamVector(confObj, saturationMin, "saturationMin"))
            return false;
        if (!ctb::GetParam(confObj, positionGainJoints, "positionGainJoints"))
            return false;
        if (!ctb::GetParam(confObj, positionGainCart, "positionGainCart"))
            return false;

        return true;
    }

    friend std::ostream& operator<<(std::ostream& os, KCLConfiguration const& a)
    {
        return os << "======= KCL CONF =======\n"
                  << "LoopRate: " << a.loopRate << "\n"
                  << "LinearErrorThreshold: " << a.linearErrorThreshold << "\n"
                  << "AngularErrorThreshold: " << a.angularErrorThreshold << "\n"
                  << "cartesianMoveTimeout: " << a.cartesianMoveTimeout << "\n"
                  << "positionGainJoints: " << a.positionGainJoints << "\n"
                  << "positionGainCart: " << a.positionGainCart << "\n"
                  << "SaturationMin: " << a.saturationMin.transpose() << "\n"
                  << "SaturationMax: " << a.saturationMax.transpose() << "\n"
                  << "===============================\n";
    }
};


#endif // CTRL_DEFINES_HPP
