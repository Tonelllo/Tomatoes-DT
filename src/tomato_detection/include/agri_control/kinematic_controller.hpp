#ifndef KINEMATIC_CONTROLLER_HPP
#define KINEMATIC_CONTROLLER_HPP

#include <unordered_map>
#include <map>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "std_msgs/Empty.h"

#include "futils.hpp"
#include "apple_robot_defines.hpp"
#include "ctrl_defines.hpp"

#include "states/state_disabled.hpp"
#include "states/state_parked.hpp"
#include "states/state_parking.hpp"
#include "states/state_unparking.hpp"
#include "states/state_idle.hpp"
#include "states/state_move_joints_pos.hpp"
#include "states/state_move_joints_vel.hpp"
#include "states/state_move_cartesian.hpp"

#include "commands/command_disable.hpp"
#include "commands/command_enable.hpp"
#include "commands/command_unpark.hpp"
#include "commands/command_park.hpp"
#include "commands/command_stop.hpp"
#include "commands/command_move_joints_pos.hpp"
#include "commands/command_move_joints_vel.hpp"
#include "commands/command_move_cartesian.hpp"

#include "tomato_detection/ControlData.h"
#include "tomato_detection/ControlCommand.h"
#include "tomato_detection/DriverCommand.h"
#include "tomato_detection/StateData.h"
#include "tomato_detection/ControlData.h"
#include "tomato_detection/TaskStatus.h"
#include "tomato_detection/TPIKPriorityLevel.h"
#include "tomato_detection/TPIKAction.h"
#include "tomato_detection/DriverCommand.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include "apple_robot_model.h"

#include <sensor_msgs/JointState.h>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>

class KinematicController {

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<KCLConfiguration> conf_;

    ros::Timer runTimer_;
    ros::Timer slow_timer_;

    ros::Subscriber stateDataSub_;

    ros::Publisher ctrlDataPub_;
    ros::Publisher tpikActionPub_;

    ros::Publisher driverCommandPub_;
    ros::Publisher goal2GazeboPub_;

    ros::ServiceServer srvCommand_;

    sensor_msgs::JointState stateDataMsg_;
    tomato_detection::ControlData controlDataMsg_;

    tomato_detection::DriverCommand driverCmd_;
    
    /// FSM
    fsm::FSM uFsm_;
    std::unordered_map<std::string, std::shared_ptr<states::GenericState>> statesMap_;
    std::unordered_map<std::string, std::shared_ptr<commands::GenericCommand>> commandsMap_;

    std::shared_ptr<commands::Disable> commandDisable_;
    std::shared_ptr<commands::Enable> commandEnable_;
    std::shared_ptr<commands::Unpark> commandUnpark_;
    std::shared_ptr<commands::Park> commandPark_;
    std::shared_ptr<commands::Stop> commandStop_;
    std::shared_ptr<commands::MoveJointsPos> commandMoveJointsPos_;
    std::shared_ptr<commands::MoveJointsVel> commandMoveJointsVel_;
    std::shared_ptr<commands::MoveCartesian> commandMoveCartesian_;

    std::shared_ptr<states::Disabled> stateDisabled_;
    std::shared_ptr<states::Parked> stateParked_;
    std::shared_ptr<states::Unparking> stateUnparking_;
    std::shared_ptr<states::Parking> stateParking_;
    std::shared_ptr<states::Idle> stateIdle_;
    std::shared_ptr<states::MoveJointsPos> stateMoveJointsPos_;
    std::shared_ptr<states::MoveJointsVel> stateMoveJointsVel_;
    std::shared_ptr<states::MoveCartesian> stateMoveCartesian_;

    /// Control
    std::shared_ptr<RobotInfo> robotInfo_;
    std::shared_ptr<rml::AppleRobotModel> armModel_;
    std::shared_ptr<rml::RobotModel> robotModel_;
    std::unordered_map<std::string, TasksInfo> tasksMap_;

    std::shared_ptr<tpik::ActionManager> actionManager_;
    std::shared_ptr<tpik::iCAT> iCat_;
    std::shared_ptr<tpik::Solver> solver_;
    Eigen::VectorXd yTpik_;

    /// UTILS
    ros::Time tLastControl_, tLastFeedback_, tSTART_;
    futils::Spinner spinner_;
    bool receivingFeedback_;
    double feedbackTimeout_;
    std::string currentState_;

    double dt_;
    double t_;
    double tPrev_;

    tf::TransformBroadcaster tfPublisher_;

    bool LoadConfiguration();
    bool Initialization();
    bool SetUpFSM();

    void StateDataCB(const sensor_msgs::JointState::ConstPtr &msg);

    bool CommandsHandler(tomato_detection::ControlCommand::Request &request,
                         tomato_detection::ControlCommand::Response &response);

    void PublishControl();
    void PublishTasksInfo();
    void PublishTrajectory();
    void SlowTimer();
    tf::Transform EigenToTF(const Eigen::TransformationMatrix &transf);
    void PublishToTF(const Eigen::TransformationMatrix &transf,
                     const std::string &frame_from, const std::string &frame_to);

    ros::Publisher jointTrajectoryPub_;

    Eigen::VectorXd targetJointPos_;

    std::vector<std::string> jointNames_;

    std::map<std::string, size_t> jointNameToIndex_;
    
    int motionId_;

    std::string id_;

    ros::Publisher velPub_;

    void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

public:
    KinematicController(std::shared_ptr<ros::NodeHandle> nodeHandle, const std::string &conf_filename,
        bool simulatedDriver, std::string prefix);

    void Run();
};


#endif // KINEMATIC_CONTROLLER_HPP
