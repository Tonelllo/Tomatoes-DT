#include "agri_control/kinematic_controller.hpp"
#include "agri_control/control_configuration.hpp"

#include <ros/package.h>
#include <tf_conversions/tf_eigen.h>

#include <agri_control/apple_robot_model.h>

#include <std_msgs/Float64MultiArray.h>

#include <sstream>                         // For std::stringstream

using std::placeholders::_1;

template<> ros::Duration ros::TimeBase<ros::Time, ros::Duration>::operator-(const ros::Time &rhs) const;

std::vector<Eigen::Vector3d> octreeToPointCloud(octomap::OcTree& tree) {
    std::vector<Eigen::Vector3d> pointCloud;  // Vector to store points

    unsigned int octree_depth_;
    auto treeDepth = sizeof(unsigned short) * 8;
    auto end = tree.end();
    for (auto it = tree.begin(treeDepth); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) pointCloud.push_back(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
    }

    return pointCloud;
}

void KinematicController::OctreeCallback(const moveit_msgs::PlanningScene::ConstPtr& msga){
    auto msg = msga->world.octomap.octomap;
    obstacleRes_ = msg.resolution;
    octree_ = new octomap::OcTree(obstacleRes_);    
    std::stringstream datastream;
    if (msg.data.size() > 0){
        datastream.write((const char*) &msg.data[0], msg.data.size());
        octree_->readData(datastream);
    }
    obstacleCentroids_ = octreeToPointCloud(*octree_);
} 

KinematicController::KinematicController(std::shared_ptr<ros::NodeHandle> nodeHandle, const std::string &conf_filename, bool isSim,
        std::string prefix)
    : nh_(nodeHandle)
    , spinner_(1)
    , receivingFeedback_(false) // TODO: put back to false
    , feedbackTimeout_(1.0)

{
    id_ = prefix;

    Initialization();
    robotInfo_->simulatedDriver =  isSim;
    std::cout << "[KCL] isSim = " << robotInfo_->simulatedDriver << std::endl;

    conf_ = std::make_shared<KCLConfiguration>(conf_filename);
    if(!LoadConfiguration()){
        ROS_ERROR_STREAM("Error in LoadConfiguration(). Exiting...");
        exit(EXIT_FAILURE);
    }

    iCat_ = std::make_shared<tpik::iCAT>(robotModel_->Dof());
    solver_ = std::make_shared<tpik::Solver>(actionManager_, iCat_);
    // Set Saturation values for the iCAT (read from conf file)
    iCat_->SetSaturation(conf_->saturationMin, conf_->saturationMax);

    SetUpFSM();

    stateDataSub_ = nh_->subscribe<sensor_msgs::JointState>("/joint_states", 10, &KinematicController::StateDataCB, this);
    octreeSub_ = nh_->subscribe<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 10, &KinematicController::OctreeCallback, this);

    ctrlDataPub_ = nh_->advertise<tomato_detection::ControlData>(prefix + appleRobot::topicnames::ctrl_data, 10);
    tpikActionPub_ = nh_->advertise<tomato_detection::TPIKAction>(prefix + appleRobot::topicnames::tpik_action, 10);

    if (isSim) armGoalPub_ = nh_->advertise<control_msgs::FollowJointTrajectoryActionGoal>(appleRobot::topicnames::armGoalSim, 10);
    else armGoalPub_ = nh_->advertise<control_msgs::FollowJointTrajectoryActionGoal>(appleRobot::topicnames::armGoal, 10);
    torsoGoalPub_ = nh_->advertise<control_msgs::FollowJointTrajectoryActionGoal>(appleRobot::topicnames::torsoGoal, 10);

    driverCommandPub_ = nh_->advertise<tomato_detection::DriverCommand>(prefix + appleRobot::topicnames::driver_cmd, 10);
    
    if (id_.find("left") != std::string::npos) {
        velPub_ = nh_->advertise<std_msgs::Float64MultiArray>("/left/joint_group_vel_controller/command", 10);
    }
    else {
        velPub_ = nh_->advertise<std_msgs::Float64MultiArray>("/right/joint_group_vel_controller/command", 10);
    }

   // cameraPosePub_ = nh_->advertise<geometric_msg::Pose>(prefix + "camPose", 10);

    // Create a service for accepting input commands
    srvCommand_ = nh_->advertiseService(prefix + appleRobot::topicnames::srv_control_command, &KinematicController::CommandsHandler, this);

    // Create the main function timer, to be exectued every 1/rate seconds
    double Ts = 1.0 / (conf_->loopRate);
    dt_ = Ts;
    std::cerr << "[KCL] Ts = " << Ts << std::endl;
    runTimer_ = nh_->createTimer(ros::Duration(Ts), std::bind(&KinematicController::Run, this));
    slow_timer_ = nh_->createTimer(ros::Duration(2), std::bind(&KinematicController::SlowTimer, this));

    std::cout << "[KCL] Loading Completed." << std::endl;

    tSTART_ = tLastFeedback_ = tLastControl_ = ros::Time::now();

    if (isSim) {
        jointNameToIndex_["torso_lift_joint"] = 0;
        jointNameToIndex_["arm_1_joint"] = 1;
        jointNameToIndex_["arm_2_joint"] = 2;
        jointNameToIndex_["arm_3_joint"] = 3;
        jointNameToIndex_["arm_4_joint"] = 4;
        jointNameToIndex_["arm_5_joint"] = 5;
        jointNameToIndex_["arm_6_joint"] = 6;
        jointNameToIndex_["arm_7_joint"] = 7;
    }
    else {
        jointNameToIndex_["torso_lift_joint"] = 0;
        jointNameToIndex_["arm_left_1_joint"] = 1;
        jointNameToIndex_["arm_left_2_joint"] = 2;
        jointNameToIndex_["arm_left_3_joint"] = 3;
        jointNameToIndex_["arm_left_4_joint"] = 4;
        jointNameToIndex_["arm_left_5_joint"] = 5;
        jointNameToIndex_["arm_left_6_joint"] = 6;
        jointNameToIndex_["arm_left_7_joint"] = 7;
    }
    jointNames_.resize(jointNameToIndex_.size());

    size_t startIdx = 0;
    for (auto jointName : jointNameToIndex_) {
        jointNames_[jointName.second] = jointName.first;
    }

    jointTrajectoryPub_ = nh_->advertise<trajectory_msgs::JointTrajectory>(prefix + appleRobot::topicnames::desired_trajectory, 1);
}

bool KinematicController::Initialization()
{
    // Create and insert states in the map
    stateDisabled_ = std::make_shared<states::Disabled>();
    stateParked_ = std::make_shared<states::Parked>();
    stateUnparking_ = std::make_shared<states::Unparking>();
    stateParking_ = std::make_shared<states::Parking>();
    stateIdle_ = std::make_shared<states::Idle>();
    stateMoveJointsPos_ = std::make_shared<states::MoveJointsPos>();
    stateMoveJointsVel_ = std::make_shared<states::MoveJointsVel>();
    stateMoveCartesian_ = std::make_shared<states::MoveCartesian>();

    statesMap_.insert({ appleRobot::ID::states::disabled   ,     stateDisabled_   });
    statesMap_.insert({ appleRobot::ID::states::parked   ,       stateParked_   });
    statesMap_.insert({ appleRobot::ID::states::unparking   ,    stateUnparking_   });
    statesMap_.insert({ appleRobot::ID::states::parking   ,      stateParking_   });
    statesMap_.insert({ appleRobot::ID::states::idle       ,     stateIdle_       });
    statesMap_.insert({ appleRobot::ID::states::move_joints_pos, stateMoveJointsPos_ });
    statesMap_.insert({ appleRobot::ID::states::move_joints_vel, stateMoveJointsVel_ });
    statesMap_.insert({ appleRobot::ID::states::move_cartesian,  stateMoveCartesian_ });

    // Create and insert commands in the map
    commandDisable_       = std::make_shared<commands::Disable>();
    commandEnable_        = std::make_shared<commands::Enable>();
    commandUnpark_        = std::make_shared<commands::Unpark>();
    commandPark_          = std::make_shared<commands::Park>();
    commandStop_          = std::make_shared<commands::Stop>();
    commandMoveJointsPos_ = std::make_shared<commands::MoveJointsPos>();
    commandMoveJointsVel_ = std::make_shared<commands::MoveJointsVel>();
    commandMoveCartesian_ = std::make_shared<commands::MoveCartesian>();

    commandsMap_.insert({ appleRobot::ID::commands::disable,         commandDisable_ });
    commandsMap_.insert({ appleRobot::ID::commands::enable,          commandEnable_ });
    commandsMap_.insert({ appleRobot::ID::commands::unpark,          commandUnpark_ });
    commandsMap_.insert({ appleRobot::ID::commands::park,            commandPark_ });
    commandsMap_.insert({ appleRobot::ID::commands::stop,            commandStop_ });
    commandsMap_.insert({ appleRobot::ID::commands::move_joints_pos, commandMoveJointsPos_ });
    commandsMap_.insert({ appleRobot::ID::commands::move_joints_vel, commandMoveJointsVel_ });
    commandsMap_.insert({ appleRobot::ID::commands::move_cartesian,  commandMoveCartesian_ });

    robotInfo_ = std::make_shared<RobotInfo>();
    actionManager_ = std::make_shared<tpik::ActionManager>();

    /// Model Setup ///

    Eigen::TransformationMatrix worldTglobal; // identity matrix for now
   //worldTglobal.RotationMatrix(rml::EulerRPY(0.0,0.0,-M_PI_2).ToRotationMatrix());
   // worldTglobal.TranslationVector(Eigen::Vector3d(0,0.10915*2,0));
    std::cerr << "worldTglobal = " << worldTglobal << std::endl;
    robotModel_ = std::make_shared<rml::RobotModel>(worldTglobal, appleRobot::ID::RobotModel::AppleRobotBody);
    if (id_.find("left") != std::string::npos) {
        armModel_ = std::make_shared<rml::AppleRobotModel>(nh_, appleRobot::ID::RobotModel::AppleRobotArmLeft);
    }
    else {
        armModel_ = std::make_shared<rml::AppleRobotModel>(nh_, appleRobot::ID::RobotModel::AppleRobotArmRight);
    }

    Eigen::VectorXd qfb_single, qdotcontrol_single;
    uint numJoints = armModel_->NumJoints();
    qfb_single = qdotcontrol_single = Eigen::VectorXd::Zero(numJoints);

    // Q_unfolded position
    std::vector<double> init_q_single = { { 0.011, 0.11, -1.4, -0.11, 1.57, 0.0, 0, 0 } };

    stateDataMsg_.position.resize(numJoints);
    for (size_t i = 0; i < numJoints; i++){
        qfb_single.at(i) = init_q_single.at(i);
        stateDataMsg_.position.at(i) = init_q_single.at(i);
    }

    // First model initialisation
    armModel_->JointsPosition(qfb_single);
    armModel_->JointsVelocity(qdotcontrol_single);

    Eigen::TransformationMatrix base_T_arm; // identity matrix for now
    base_T_arm.RotationMatrix(rml::EulerRPY(0.0,0.0,0.0).ToRotationMatrix());
    if (id_.find("left") != std::string::npos) {
        base_T_arm.TranslationVector(Eigen::Vector3d(0,0,0));
    }

    std::cerr << "base_T_arm = " << base_T_arm << std::endl;

    robotModel_->LoadArm(armModel_, base_T_arm);
    
    int dof = robotModel_->Dof();

    // Initialize ActionManager iCAT
    yTpik_ = Eigen::VectorXd::Zero(dof);

    /// TPIK Setup ///
    // Attach a tool frame to the last joint
    std::string jointNID = armModel_->ID() + rml::FrameID::Joint + std::to_string(dof - 1);
    Eigen::TransformationMatrix jointN_T_toolF;
    jointN_T_toolF.TranslationVector(Eigen::Vector3d(0.0,0,0.197)); // rosrun tf tf_echo /arm_7_link /gripper_grasping_frame
    Eigen::RotationMatrix toolRot = (rml::EulerRPY(0, 1.571, 0)).ToRotationMatrix();
    jointN_T_toolF.RotationMatrix(toolRot);

    robotModel_->AttachRigidBodyFrame(appleRobot::ID::RobotModel::ToolFrame, jointNID, jointN_T_toolF);
    robotInfo_->toolID = armModel_->ID() + "_" + appleRobot::ID::RobotModel::ToolFrame;

    //std::cerr << "worldTbase" << std::endl;
    //std::cerr << armModel_->TransformationMatrix(rml::FrameID::WorldFrame, appleRobot::ID::RobotModel::AppleRobotArmLeft);
    
    ////// SETTING UP TASKS //////
    TasksInfo taskInfo;

    // JOINTS LIMIT
    taskInfo.task = std::make_shared<ikcl::JointsLimit>(appleRobot::ID::Tasks::JointsLimit, robotModel_, armModel_->ID());
    taskInfo.taskPub = nh_->advertise<tomato_detection::TaskStatus>(appleRobot::topicnames::tasks + appleRobot::ID::Tasks::JointsLimit, 1);
    tasksMap_.insert(std::make_pair(appleRobot::ID::Tasks::JointsLimit, taskInfo));

    // SELF COLLISION AVOIDANCE
    std::string jointBaseID = armModel_->ID() + rml::FrameID::Joint + std::to_string(0);
    armModel_->TransformationMatrix(jointBaseID);
    //std::cerr << "[KCL::Initialization] armModel_->TransformationMatrix(jointBaseID) = " << armModel_->TransformationMatrix(jointBaseID) << std::endl;
    /*rml::PlaneParameters avoidancePlaneParams;
    avoidancePlaneParams.C = -1.0;
    avoidancePlaneParams.D = 0.15;
    ikcl::PlaneObstacle avoidancePlane(avoidancePlaneParams, jointBaseID);*/

    rml::PlaneParameters groundPlaneParams;
    nh_->getParam("/ground_plane/a", groundPlaneParams.A);
    nh_->getParam("/ground_plane/b", groundPlaneParams.B);
    nh_->getParam("/ground_plane/c", groundPlaneParams.C);
    nh_->getParam("/ground_plane/d", groundPlaneParams.D);
    ikcl::PlaneObstacle groundPlane(groundPlaneParams, rml::FrameID::WorldFrame);
    
    Eigen::TransformationMatrix worldF_T_testSphere;
    worldF_T_testSphere.TranslationVector(Eigen::Vector3d(0.0,0,0.197));
    ikcl::SphereObstacle testSphere(worldF_T_testSphere, rml::FrameID::WorldFrame, 20.0);

    std::vector<std::string> framesID; 
    framesID.push_back(robotInfo_->toolID);
    //framesID.push_back( robotInfo_->toolID);
    std::vector<std::shared_ptr<ikcl::Obstacle>> obstacles;
   // obstacles.push_back(std::make_shared<ikcl::PlaneObstacle>(groundPlane));
   // obstacles.push_back(std::make_shared<ikcl::SphereObstacle>(testSphere));
    std::cerr << tc::greenL << "[KinematicCtrler] obstacles size = " << obstacles.size() << std::endl;

    try {
        std::cerr << tc::greenL << "[KinematicCtrler] obstacles size = " << obstacles.size() << std::endl;
        std::shared_ptr<ikcl::ObstacleAvoidance> avoidanceTask
            = std::make_shared<ikcl::ObstacleAvoidance>(appleRobot::ID::Tasks::ObstacleAvoidance, robotModel_, framesID, obstacles);
        taskInfo.task = avoidanceTask;
        taskInfo.taskPub = nh_->advertise<tomato_detection::TaskStatus>(appleRobot::topicnames::tasks + appleRobot::ID::Tasks::ObstacleAvoidance, 1);
        tasksMap_.insert(std::make_pair(appleRobot::ID::Tasks::ObstacleAvoidance, taskInfo));

    } catch (rml::WrongFrameException& ex) {
        std::cout << tc::redL << "ex.how(): " << ex.how() << tc::none << std::endl;
    }

    // JOINTS POSITION
    taskInfo.task = std::make_shared<ikcl::JointsPosition>(appleRobot::ID::Tasks::JointsPosition, robotModel_, armModel_->ID());
    taskInfo.taskPub = nh_->advertise<tomato_detection::TaskStatus>(appleRobot::topicnames::tasks + appleRobot::ID::Tasks::JointsPosition, 1);
    tasksMap_.insert(std::make_pair(appleRobot::ID::Tasks::JointsPosition, taskInfo));

    // JOINTS VELOCITY
    taskInfo.task = std::make_shared<ikcl::JointsVelocity>(appleRobot::ID::Tasks::JointsVelocity, robotModel_, armModel_->ID());
    taskInfo.taskPub = nh_->advertise<tomato_detection::TaskStatus>(appleRobot::topicnames::tasks + appleRobot::ID::Tasks::JointsVelocity, 1);
    tasksMap_.insert(std::make_pair(appleRobot::ID::Tasks::JointsVelocity, taskInfo));

    // TOOL ASV CONTROL DISTANCE
    taskInfo.task = std::make_shared<ikcl::CartesianDistance>(appleRobot::ID::Tasks::ToolCartesianDistance, robotModel_, robotInfo_->toolID);
    taskInfo.taskPub = nh_->advertise<tomato_detection::TaskStatus>(appleRobot::topicnames::tasks + appleRobot::ID::Tasks::ToolCartesianDistance, 1);
    tasksMap_.insert(std::make_pair(appleRobot::ID::Tasks::ToolCartesianDistance, taskInfo));

    // TOOL CONTROL ANGULAR POSITION
    taskInfo.task = std::make_shared<ikcl::CartesianOrientation>(appleRobot::ID::Tasks::ToolCartesianOrientation, robotModel_, robotInfo_->toolID);
    taskInfo.taskPub = nh_->advertise<tomato_detection::TaskStatus>(appleRobot::topicnames::tasks + appleRobot::ID::Tasks::ToolCartesianOrientation, 1);
    tasksMap_.insert(std::make_pair(appleRobot::ID::Tasks::ToolCartesianOrientation, taskInfo));

    // Msgs
    controlDataMsg_.joint_jog.joint_names.resize(static_cast<uint>(dof));
    controlDataMsg_.joint_jog.velocities.resize(static_cast<uint>(dof));

    controlDataMsg_.header.frame_id = armModel_->ID();
    for (size_t i = 0; i < numJoints; i++){
        controlDataMsg_.joint_jog.joint_names.at(i) = rml::FrameID::Joint + std::to_string(i);
        controlDataMsg_.joint_jog.velocities.at(i) = 0.15;
    }

    std::cout << "controlDataMsg_.joint_jog.velocities: " << futils::STLVectorToString(controlDataMsg_.joint_jog.velocities,',') << std::endl;

    robotInfo_->parkedJointsPos.resize(numJoints);
    robotInfo_->unparkedJointsPos.resize(numJoints);

    return true;
}

bool KinematicController::LoadConfiguration()
{
    /////////////////////////////////////////////////
    /////        LOAD KCL CONFIGURATION         /////
    /////////////////////////////////////////////////
    std::string confPath = ros::package::getPath("tomato_detection");
    confPath.append("/conf/");
    confPath.append(conf_->filename);

    std::cout << "[KCL] PATH TO KCL CONF FILE : " << confPath << std::endl;

    // Read the configuration file. If there is an error, report it and exit.
    libconfig::Config confObj;
    
    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return false;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return false;
    }

    if (!conf_->ConfigureFromFile(confObj)) {
        std::cerr << "Failed to load KCL configuration" << std::endl;
        return false;
    } else {
        std::cout << tc::brown << *conf_ << tc::none << std::endl;
    }

    std::cout << "[KCL] Configuring Tasks, Priority Levels and Actions..." << std::endl;

    if (!ConfigureTasksFromFile(tasksMap_, confObj)) {
        std::cerr << "Failed to load Tasks from file" << std::endl;
        return false;
    };
    std::cout << tc::redL << "[KCL] Tasks configured." << tc::none << std::endl;

    if (!ConfigurePriorityLevelsFromFile(actionManager_, tasksMap_, confObj)) {
        std::cerr << "Failed to load Priority Levels from file" << std::endl;
        return false;
    };
    std::cout << tc::green << "[KCL] Priority Levels configured." << tc::none << std::endl;

    if (!ConfigureActionsFromFile(actionManager_, confObj)) {
        std::cerr << "Failed to load  Actions from file" << std::endl;
        return false;
    };
    std::cout << tc::green << "[KCL] Actions configured." << tc::none << std::endl;

    return true;
}


bool KinematicController::SetUpFSM()
{
    // ***** COMMANDS ***** //
    for (auto& command : commandsMap_) {
        command.second->robotInfo = robotInfo_;
        command.second->SetFSM(&uFsm_);
        command.second->SetNodeHandle(nh_);
    }

    commandEnable_->SetState(stateIdle_);
    commandStop_->SetState(stateIdle_);
    commandUnpark_->SetState(stateUnparking_);
    commandPark_->SetState(stateParking_);
    commandDisable_->SetState(stateDisabled_);
    commandMoveJointsPos_->SetState(stateMoveJointsPos_);
    commandMoveJointsVel_->SetState(stateMoveJointsVel_);
    commandMoveCartesian_->SetState(stateMoveCartesian_);


    // ***** STATES ***** //
    //Set the fsm and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->actionManager = actionManager_;
        state.second->robotModel = robotModel_;
        state.second->tasksMap = tasksMap_;
        state.second->robotInfo = robotInfo_;
        state.second->conf = conf_;
        state.second->SetFSM(&uFsm_);
    }

    // (Should be implemented in a smarter way)
    stateMoveJointsPos_->armModel = armModel_;
    stateMoveJointsVel_->armModel = armModel_;
    stateUnparking_->armModel = armModel_;
    stateParking_->armModel = armModel_;

    // ***** FSM CONFIGURATION ***** //
    // ADD COMMANDS
    for (auto& command : commandsMap_) {
        uFsm_.AddCommand(command.first, command.second.get());
    }

    // ADD STATES
    for (auto& state : statesMap_) {
        uFsm_.AddState(state.first, state.second.get());
    }

    // ENABLE TRANSITIONS
    for (auto& currentState : statesMap_) {
        for (auto& nextState : statesMap_) {
            if (nextState.first != currentState.first)
                uFsm_.EnableTransition(currentState.first, nextState.first, true);
        }
    }

    uFsm_.EnableCommandInState(appleRobot::ID::states::disabled, appleRobot::ID::commands::enable, true);

    // We can disable, stop, unpark the robot in any state
    for (auto& state : statesMap_) {
        uFsm_.EnableCommandInState(state.first, appleRobot::ID::commands::disable, true);
        uFsm_.EnableCommandInState(state.first, appleRobot::ID::commands::stop, true);

    }

    // We can send any command when in idle
    for (auto& command : commandsMap_) {
        uFsm_.EnableCommandInState(appleRobot::ID::states::idle, command.first, true);
    }

    uFsm_.EnableCommandInState(appleRobot::ID::states::parked, appleRobot::ID::commands::unpark, true);

    uFsm_.EnableCommandInState(appleRobot::ID::states::idle, appleRobot::ID::commands::park, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_joints_pos, appleRobot::ID::commands::park, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_joints_vel, appleRobot::ID::commands::park, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_cartesian, appleRobot::ID::commands::park, true);

    uFsm_.EnableCommandInState(appleRobot::ID::states::move_joints_pos, appleRobot::ID::commands::move_joints_pos, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_joints_vel, appleRobot::ID::commands::move_joints_pos, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_cartesian, appleRobot::ID::commands::move_joints_pos, true);

    uFsm_.EnableCommandInState(appleRobot::ID::states::move_joints_pos, appleRobot::ID::commands::move_joints_vel, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_joints_vel, appleRobot::ID::commands::move_joints_vel, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_cartesian, appleRobot::ID::commands::move_joints_vel, true);

    uFsm_.EnableCommandInState(appleRobot::ID::states::move_joints_vel, appleRobot::ID::commands::move_cartesian, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_joints_pos, appleRobot::ID::commands::move_cartesian, true);
    uFsm_.EnableCommandInState(appleRobot::ID::states::move_cartesian, appleRobot::ID::commands::move_cartesian, true);

    if (robotInfo_->simulatedDriver) {
        uFsm_.EnableCommandInState(appleRobot::ID::states::parked, appleRobot::ID::commands::move_joints_pos, true);
    }

    // ENABLE ALL COMMANDS
    for (auto& state : statesMap_) {
        for (auto& command : commandsMap_) {
           uFsm_.EnableCommandInState(state.first, command.first, true);
        }
    }

    uFsm_.SetInitState(appleRobot::ID::states::disabled);

    return true;
}


void KinematicController::SlowTimer()
{

    if (receivingFeedback_) {

        // Publish Hierarchy Info
        tomato_detection::TPIKAction tpikActionMsg_;
        tpikActionMsg_.id = actionManager_->CurrentActionID();
        tpik::Hierarchy hierarchy = actionManager_->GetAction(tpikActionMsg_.id)->PriorityLevels();

        for (size_t i = 0; i < hierarchy.size(); i++) {

            tpikActionMsg_.priority_levels.push_back(tomato_detection::TPIKPriorityLevel());
            tpikActionMsg_.priority_levels.at(i).id = hierarchy.at(i)->ID();
            std::vector<std::shared_ptr<tpik::Task>> tasks = hierarchy.at(i)->Level();

            for (size_t j = 0; j < tasks.size(); j++) {

                tpikActionMsg_.priority_levels.at(i).tasks_id.push_back(tasks.at(j)->ID());
            }
        }
        tpikActionPub_.publish(tpikActionMsg_);
    }

    std::cout << tc::mag << id_ << "[KCL] Current TPIK Action: " << actionManager_->CurrentActionID() << "." << tc::none << std::endl;
    std::cerr << id_ << "[KCL] ytpik = " <<  yTpik_.transpose() << std::endl;
    std::cerr << id_ << "[KCL] believed joint position = " << armModel_->JointsPosition().transpose()  << std::endl;
    std::cerr << id_ << "[KCL] receivingFeedback_ = " << receivingFeedback_ << std::endl;
    std::cerr << id_ << "[KCL] world_T_tool = " << robotModel_->TransformationMatrix(rml::FrameID::WorldFrame, robotInfo_->toolID) << std::endl;
    std::cerr << id_ << "[KCL] targetJointPos = " << targetJointPos_.transpose() << std::endl;
}

void KinematicController::Run()
{
    spinner_();

    // Switch State (if something happens)
    uFsm_.SwitchState();

    if(uFsm_.GetCurrentStateName() != currentState_) {
        currentState_ = uFsm_.GetCurrentStateName();
        std::cout << tc::mag << "[KCL] State: " << currentState_ << "." << tc::none << std::endl;
    }

    // Process Events
    uFsm_.ProcessEventQueue();
    // Execute current state
    uFsm_.ExecuteState();

    if (receivingFeedback_) {

        // PublishToTF(robotModel_->TransformationMatrix(robotInfo_->toolID), "base_", "tool");
        auto stateIsIdle = false; // TODO put true state feedback from state machine
        auto stateIsDisabled = false; // TODO put true state feedback from state machine

        if (uFsm_.GetCurrentStateName() != appleRobot::ID::states::disabled) {

            auto worldF_tool = robotModel_->TransformationMatrix(robotInfo_->toolID).TranslationVector();

            /*std::vector<std::shared_ptr<ikcl::Obstacle>> obstacles;
            for (auto cntr : obstacleCentroids_) {
                if ((cntr - worldF_tool).norm() < 0.5) {
                    Eigen::TransformationMatrix worldF_T_sphere;
                    worldF_T_sphere.TranslationVector(cntr);
                    ikcl::SphereObstacle obstacleSphere(worldF_T_sphere, rml::FrameID::WorldFrame, obstacleRes_/2.0);
                    obstacles.push_back(std::make_shared<ikcl::SphereObstacle>(obstacleSphere));
                   // if (obstacles.size() > 5) break;
                }
            }
            if (tasksMap_.find(appleRobot::ID::Tasks::ObstacleAvoidance) != tasksMap_.end()) {
                auto task = tasksMap_[appleRobot::ID::Tasks::ObstacleAvoidance].task;
                std::shared_ptr<ikcl::ObstacleAvoidance> obstacleAvoidanceTask = std::dynamic_pointer_cast<ikcl::ObstacleAvoidance>(task);

                if (obstacleAvoidanceTask != nullptr) {
                    obstacleAvoidanceTask->Obstacles(obstacles) ;
                    tasksMap_[appleRobot::ID::Tasks::ObstacleAvoidance].task = obstacleAvoidanceTask;
                }
            }*/

            for (auto& taskMap : tasksMap_) {
                try {
                    taskMap.second.task->Update();
                    //std::cerr << "[KCL::Run] Update task map!" << std::endl;
                } catch (tpik::ExceptionWithHow& e) {
                    std::cerr << "task is "<< taskMap.second.task->ID() << std::endl;
                    std::cerr << tc::redL << "[UPDATE TASK EXCEPTION]" << tc:: none << std::endl;
                    std::cerr << "what: " << e.what() << ", how: " << e.how() << std::endl;
                }
            }

            // CHECK IF THERE IS STILL INCOMING FEEDBACK
            auto tNow = ros::Time::now();
            double dtFbk = (tNow - tLastFeedback_).toSec();
            if (dtFbk > feedbackTimeout_) {
                receivingFeedback_ = false;
                ROS_WARN_STREAM("Feedback not received for more than " << std::to_string(feedbackTimeout_) << " seconds. Switching to Idle.");
                std::cerr << "Feedback not received for more than " << std::to_string(feedbackTimeout_) << " seconds. Switching to Idle." << std::endl;
                //uFsm_.SetNextState(appleRobot::ID::states::idle); // TODO check if it makes sense to use this line
            } else {
                if (uFsm_.GetCurrentStateName() == appleRobot::ID::states::idle) {
                    //std::cerr << "[KCL::Run] TPIK solver is zeroing!" << std::endl;
                    yTpik_.setZero();
                } else {
                   // std::cerr << "[KCL::Run] TPIK solver is solving!" << std::endl;
                    // Computing Kinematic Control via TPIK
                    yTpik_ = solver_->ComputeVelocities();
                   // std::cerr << "[KCL::Run] ytpik = " << yTpik_.transpose() << std::endl;

                    //auto deltays = solver_->DeltaYs();
                    //int i = 0;
                    //Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", " / ", "", "", "", ";");
                    //for (auto& deltay : deltays) {
                    //    std::cout << "deltay_" << i << ": " << deltay.transpose().format(CommaInitFmt) << std::endl;
                    //    i++;
                    //}
                    for (int i = 0; i < yTpik_.size(); i++) {
                        if (std::isnan(yTpik_(i))) {
                            yTpik_(i) = 0.0;
                            ROS_INFO("NaN requested velocity");
                        }
                    }
                }
            }
        }

        if (robotInfo_->simulatedDriver) {
            if (true) {
                yTpik_ = solver_->ComputeVelocities();
            }
        }

        PublishControl();
        PublishTasksInfo();
        PublishTrajectory();

        //else {
        //   ROS_WARN_STREAM_ONCE("Controller started. Arm Disabled.");
        //}
    } else {
        ROS_WARN_STREAM_ONCE("[KCL] Waiting for feedback...");
    }
}

void KinematicController::PublishTrajectory() {
    trajectory_msgs::JointTrajectory jointTrajectoryMsg, torsoTrajectoryMsg;
    jointTrajectoryMsg.header.stamp = ros::Time::now();
    targetJointPos_ = yTpik_ * dt_ + armModel_->JointsPosition();

    // PUBLISH ARM JOINT GOAL
    auto armJointNames = jointNames_;
    armJointNames.erase(
        std::remove_if(armJointNames.begin(), armJointNames.end(),
            [](const std::string& name) {
                return name.find("torso") != std::string::npos;  // Check if "torso" is in the string
            }
        ),
        armJointNames.end()
    );
    jointTrajectoryMsg.joint_names = armJointNames;
    
    trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint;
    jointTrajectoryPoint.positions.resize(armJointNames.size(),0.0);
    jointTrajectoryPoint.velocities.resize(armJointNames.size(),0.0);
    jointTrajectoryPoint.accelerations.resize(armJointNames.size(),0.0);
    jointTrajectoryPoint.effort.resize(armJointNames.size(),0.0);
    
    for (auto i = 0; i < armJointNames.size(); i++) {
        if (jointNameToIndex_.find(armJointNames.at(i)) != jointNameToIndex_.end()) {
            auto j = jointNameToIndex_.at(armJointNames.at(i));
            jointTrajectoryPoint.positions[i] = targetJointPos_[j];
        }
    }

    jointTrajectoryPoint.time_from_start = ros::Duration(dt_);
    jointTrajectoryMsg.points.push_back(jointTrajectoryPoint);
    //std::cerr << std::endl;

    control_msgs::FollowJointTrajectoryActionGoal msg;
    msg.goal.trajectory = jointTrajectoryMsg;
    armGoalPub_.publish(msg);

    // PUBLISH TORSO GOAL
    trajectory_msgs::JointTrajectoryPoint torsoTrajectoryPoint;
    torsoTrajectoryMsg.joint_names = {"torso_lift_joint"};
    torsoTrajectoryPoint.positions.resize(1,0.0);
    torsoTrajectoryPoint.velocities.resize(1,0.0);
    torsoTrajectoryPoint.accelerations.resize(1,0.0);
    torsoTrajectoryPoint.effort.resize(1,0.0);

    torsoTrajectoryPoint.positions[0] = targetJointPos_[jointNameToIndex_.at(torsoTrajectoryMsg.joint_names[0])];

    torsoTrajectoryPoint.time_from_start = ros::Duration(dt_);
    torsoTrajectoryMsg.points.push_back(torsoTrajectoryPoint);

    control_msgs::FollowJointTrajectoryActionGoal torsoMsg;
    torsoMsg.goal.trajectory = torsoTrajectoryMsg;
    torsoGoalPub_.publish(torsoMsg);
}


bool KinematicController::CommandsHandler(tomato_detection::ControlCommand::Request &request,
                                          tomato_detection::ControlCommand::Response &response)
{

    // Callback function for when service requests are received.
    fsm::retval ret = fsm::ok;

    std::cerr << "[KCL::CommandsHandler] Entering.." << std::endl;

    if (receivingFeedback_) {
        //std::cerr << "[KinematicController::CommandsHandler] We have feedback!" << std::endl;
        if (request.command_type == appleRobot::ID::commands::disable)
        {
        }
        else if (request.command_type == appleRobot::ID::commands::unpark)
        {
        }
        else if (request.command_type == appleRobot::ID::commands::park)
        {
        }
        else if (request.command_type == appleRobot::ID::commands::stop)
        {
            uFsm_.SetNextState(appleRobot::ID::states::idle);
        }
        else if (request.command_type == appleRobot::ID::commands::move_joints_pos)
        {
            std::cerr << "robot ID = " << id_ << std::endl;
                
            motionId_ = request.id;
            
            request.joint_setpoint[0] = armModel_->jointDirectionSwapSim.at(0) * request.joint_setpoint[0] - armModel_->jointOffsetsSim.at(0);
            request.joint_setpoint[1] = armModel_->jointDirectionSwapSim.at(1) * request.joint_setpoint[1] - armModel_->jointOffsetsSim.at(1);
            request.joint_setpoint[2] = armModel_->jointDirectionSwapSim.at(2) * request.joint_setpoint[2] - armModel_->jointOffsetsSim.at(2);
            request.joint_setpoint[3] = armModel_->jointDirectionSwapSim.at(3) * request.joint_setpoint[3] - armModel_->jointOffsetsSim.at(3);
            request.joint_setpoint[4] = armModel_->jointDirectionSwapSim.at(4) * request.joint_setpoint[4] - armModel_->jointOffsetsSim.at(4);
            request.joint_setpoint[5] = armModel_->jointDirectionSwapSim.at(5) * request.joint_setpoint[5] - armModel_->jointOffsetsSim.at(5);

            commandMoveJointsPos_->SetJointsGoal(request.joint_setpoint, request.move_type);
            commandMoveJointsPos_->enableObstacleAvoidance = request.enableObstacleAvoidance;
        }
        else if (request.command_type == appleRobot::ID::commands::move_joint_i_pos)
        {
            motionId_ = request.id;
            commandMoveJointsPos_->SetSingleJointGoal(request.joint_setpoint, request.joint_index, request.move_type);
            request.command_type = appleRobot::ID::commands::move_joints_pos;
        }
        else if (request.command_type == appleRobot::ID::commands::move_joints_vel)
        {
            motionId_ = request.id;
            commandMoveJointsVel_->SetJointsGoal(request.joint_setpoint);
        }
        else if (request.command_type == appleRobot::ID::commands::move_joint_i_vel)
        {
            motionId_ = request.id;
            commandMoveJointsVel_->SetSingleJointGoal(request.joint_setpoint, request.joint_index);
            request.command_type = appleRobot::ID::commands::move_joints_vel;
        }
        else if (request.command_type == appleRobot::ID::commands::move_cartesian)
        {
            motionId_ = request.id;
            commandMoveCartesian_->SetCartesianGoal(request.target_position, request.target_orientation,
                                                    request.move_type, request.frame_type);
        }
        else if (request.command_type == appleRobot::ID::commands::gripper_move)
        {
            driverCmd_.command_id = appleRobot::ID::GTManipulatorCommand::GRIPDESPOS;
            driverCmd_.value = request.gripper_setpoint;
            driverCommandPub_.publish(driverCmd_);
        }
        else if (request.command_type == appleRobot::ID::commands::gripper_grasp)
        {
            driverCmd_.command_type = appleRobot::ID::GTManipulatorCommand::GRIPCMD;
            driverCmd_.command_id = appleRobot::ID::GTGripperCommand::SETCURR;
            driverCmd_.value = request.grasp_current;
            driverCommandPub_.publish(driverCmd_);

            driverCmd_.command_type = appleRobot::ID::GTManipulatorCommand::GRIPDESPOS;
            driverCmd_.value = GRIPPER_CLOSED_SETPOINT;
            driverCommandPub_.publish(driverCmd_);

        } else
        {
            response.res = "CommandAnswer::fail - Unsupported command: " + request.command_type;
           // std::cerr << "[KCL::CommandsHandler] Unsupp command!" << std::endl;
            ret = fsm::retval::fail;
        }
    } else {
        response.res = "CommandAnswer::fail - NO Feedback, discarding command.";
        ret = fsm::retval::fail;
    }

    if (ret == fsm::retval::ok) {
        // Task update
        for (auto& taskMap : tasksMap_) {
            try {
                taskMap.second.task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                ROS_ERROR_STREAM("UPDATE TASK EXCEPTION");
                ROS_ERROR_STREAM("who " << e.what() << " how: " << e.how());
            }
        }

        std::cerr << "[KinematicController::CommandsHandler] request.command_type = " << request.command_type << std::endl;
        if ( uFsm_.ExecuteCommand(request.command_type) == fsm::ok ){
            response.res = "Executing: " + request.command_type;
         //   std::cerr << "[KinematicController::CommandsHandler] Execution ok!" << std::endl;
        } else {
            response.res = "No Command Executed: FSM Conflict ";
          //  std::cerr << "[KinematicController::CommandsHandler] Execution NOT ok!" << std::endl;
        }
    }

    ROS_INFO_STREAM(tc::greenL << "[KCL] Received Command: " << request.command_type
                               << " | Response: " << response.res << tc::none);
    return true;
}

void KinematicController::PublishControl()
{
    tLastControl_ = ros::Time::now();

    controlDataMsg_.header.stamp = tLastControl_;

    controlDataMsg_.ctrl_state = uFsm_.GetCurrentStateName();
    controlDataMsg_.jointPos_ok = stateMoveJointsPos_->ok_;
    controlDataMsg_.cart_ok = stateMoveCartesian_->ok_;

    controlDataMsg_.idMotion = motionId_ ;

    auto stateIsDisabled = false; // TODO put true state feedback from state machine
    if (!stateIsDisabled) {

        Eigen::TransformationMatrix w_T_tool = robotModel_->TransformationMatrix(robotInfo_->toolID);

        controlDataMsg_.tool_position_xyz.at(0) = w_T_tool.TranslationVector().x();
        controlDataMsg_.tool_position_xyz.at(1) = w_T_tool.TranslationVector().y();
        controlDataMsg_.tool_position_xyz.at(2) = w_T_tool.TranslationVector().z();

        controlDataMsg_.tool_orientation_rpy.at(0) = w_T_tool.RotationMatrix().ToEulerRPY().Roll();
        controlDataMsg_.tool_orientation_rpy.at(1) = w_T_tool.RotationMatrix().ToEulerRPY().Pitch();
        controlDataMsg_.tool_orientation_rpy.at(2) = w_T_tool.RotationMatrix().ToEulerRPY().Yaw();

        //std::cout << "controlData_.ctrl_state: " << controlData_.ctrl_state << std::endl;

        for(size_t i = 0; i < armModel_->NumJoints(); i++){
            controlDataMsg_.joint_jog.velocities.at(i) = yTpik_.at(i);
        }
    }
    ctrlDataPub_.publish(controlDataMsg_);

    // Publish relevant frames to /tf



    //if (uFsm_.GetCurrentStateName() == appleRobot::ID::states::move_cartesian) {
    //    PublishToTF(robotInfo_->cartesianGoalFrame, "base", "goal");
    //} // TODO check if it makes sense to use this 

}


void KinematicController::PublishTasksInfo()
{

    // Publish Tasks Information
    for (auto& taskInfo : tasksMap_) {
        try {

            std::vector<double> diagonal_internal_activation_function;
            for (unsigned int i = 0; i < taskInfo.second.task->InternalActivationFunction().rows(); i++) {
                diagonal_internal_activation_function.push_back(taskInfo.second.task->InternalActivationFunction().at(i, i));
            }

            std::vector<double> diagonal_external_activation_function;
            for (unsigned int i = 0; i < taskInfo.second.task->ExternalActivationFunction().rows(); i++) {
                diagonal_external_activation_function.push_back(taskInfo.second.task->ExternalActivationFunction().at(i, i));
            }

            std::vector<double> referenceRate;
            for (unsigned int i = 0; i < taskInfo.second.task->ReferenceRate().size(); i++) {
                referenceRate.push_back(taskInfo.second.task->ReferenceRate().at(i));
            }

            tomato_detection::TaskStatus taskstatus_msg;

            taskstatus_msg.stamp.data = tLastControl_;

            taskstatus_msg.id = taskInfo.second.task->ID();
            taskstatus_msg.in_current_action = actionManager_->IsTaskInCurrentAction(taskInfo.second.task->ID());
            taskstatus_msg.enabled = taskInfo.second.task->Enabled();
            taskstatus_msg.external_activation_function = diagonal_external_activation_function;
            taskstatus_msg.internal_activation_function = diagonal_internal_activation_function;
            taskstatus_msg.reference_rate = referenceRate;

            taskInfo.second.taskPub.publish(taskstatus_msg);

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "LOG TASK EXCEPTION" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }
}

void KinematicController::StateDataCB(const sensor_msgs::JointState::ConstPtr &msg)
{
    stateDataMsg_ = *msg;

    //std::cerr << "KinematicController is receiving feedback!" << std::endl;

    //robotInfo_->arm_state_id = stateDataMsg_.arm_state_id;
    //robotInfo_->gripper_state_id = stateDataMsg_.gripper_state_id;

    tLastFeedback_ = stateDataMsg_.header.stamp;

    Eigen::VectorXd armPos(armModel_->NumJoints());
    Eigen::VectorXd armVel(armModel_->NumJoints());

    for (auto fbkIdx = 0; fbkIdx < stateDataMsg_.name.size(); fbkIdx++) {
        auto jointName = stateDataMsg_.name[fbkIdx];
        if (jointNameToIndex_.find(jointName) != jointNameToIndex_.end()) {
            auto myJointIdx = jointNameToIndex_[jointName];
            armPos(myJointIdx) = stateDataMsg_.position.at(fbkIdx);
            armVel(myJointIdx) = stateDataMsg_.velocity.at(fbkIdx);
            //std::cerr << tc::bluL << "joint n " << myJointIdx << " has vel " << armVel(myJointIdx) << tc::none << std::endl;
        }
    }

    armModel_->JointsPosition(armPos);
    armModel_->JointsVelocity(armVel);

    if(!receivingFeedback_){
        ROS_INFO_STREAM("[KCL] Incoming Feedback.");
        receivingFeedback_ = true;
    }

    //std::cerr << "[KinematicController::State,DataCB] Finished!" << std::endl;
}

tf::Transform KinematicController::EigenToTF(const Eigen::TransformationMatrix &transf)
{
    tf::Transform transform;
    //Eigen::TransformationMatrix toolFrame = armModel_->TransformationMatrix(ctrlInfo_->toolID);
    Eigen::Vector3d tPos = transf.TranslationVector();
    transform.setOrigin( tf::Vector3(tPos.x(), tPos.y(), tPos.z()) );

    Eigen::Quaterniond eq(transf.RotationMatrix());
    tf::Quaternion q;
    tf::quaternionEigenToTF(eq, q);
    transform.setRotation(q);

    return transform;
}


void KinematicController::PublishToTF(const Eigen::TransformationMatrix &transf, const std::string &frame_from, const std::string &frame_to)
{

    tfPublisher_.sendTransform(tf::StampedTransform(EigenToTF(transf), ros::Time::now(), frame_from, frame_to));

}
