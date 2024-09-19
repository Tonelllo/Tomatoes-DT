#include "states/state_move_joints_pos.hpp"



namespace states {

MoveJointsPos::MoveJointsPos() { }

MoveJointsPos::~MoveJointsPos() { }

bool MoveJointsPos::ConfigureStateFromFile(libconfig::Config& confObj)
{
    (void) confObj;
    //const libconfig::Setting& root = confObj.getRoot();
    //const libconfig::Setting& states = root["states"];

    /*const libconfig::Setting& state = states.lookup(appleRobot::ID::states::halt);
    if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
        return false;
    if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
        return false;*/

    return true;
}

bool MoveJointsPos::SetJointsGoal(std::vector<double> &joints_pos, std::string &move_type)
{

    if (joints_pos.size() == armModel->NumJoints()) {
        Eigen::Map<Eigen::VectorXd> jointsGoal(joints_pos.data(), robotModel->Dof());

        if (move_type == tomato_detection::ControlCommandRequest::ABSMOVE) {
            robotInfo->jointsGoal = jointsGoal;
        } else if (move_type == tomato_detection::ControlCommandRequest::RELMOVE) {
            robotInfo->jointsGoal = armModel->JointsPosition() + jointsGoal;
        } else {
            ROS_ERROR("[MoveJointsPos] Wrong move type");
            return false;
        }
        moveTimer_.Start();
        return true;
    } else {
        return false;
    }
}

bool MoveJointsPos::SetSingleJointGoal(std::vector<double> &joint_pos, int i, std::string &move_type) {

    if (joint_pos.size() == 1) {
        robotInfo->jointsGoal = armModel->JointsPosition();

        if (move_type == tomato_detection::ControlCommandRequest::ABSMOVE) {
            robotInfo->jointsGoal(i) = joint_pos.at(0);
        } else if (move_type == tomato_detection::ControlCommandRequest::RELMOVE) {
            robotInfo->jointsGoal(i) += joint_pos.at(0);
        } else {
            ROS_ERROR("[MoveJointsPos] Wrong move type");
            return false;
        }
        //std::cout << "jointsGoal: " << jointsGoal.transpose() << std::endl;
        return true;
    } else {
        return false;
    }

}

fsm::retval MoveJointsPos::OnEntry()
{

    jointsPositionTask_ = std::dynamic_pointer_cast<ikcl::JointsPosition>(tasksMap.find(appleRobot::ID::Tasks::JointsPosition)->second.task);
    ok_ = true;

    if (!enableObstacleAvoidance) {
        auto eaf = tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction();
        tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction() = eaf.setZero();
        // std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![MoveJointsPos::OnEntry] tmp eaf after zeroing = " << tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction() << std::endl;
    }
    if (actionManager->SetAction(appleRobot::ID::Actions::MoveJointsPosition, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }
}

fsm::retval MoveJointsPos::OnExit()
{
    moveTimer_.Stop();
    auto eaf = tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction();
    tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction() = eaf.setIdentity();
    //std::cerr << "[MoveJointsPos::OnEntry] tmp eaf after oneing = " << tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction() << std::endl;
    return fsm::ok;
}


fsm::retval MoveJointsPos::Execute()
{
    Eigen::RotationMatrix rotTool = robotModel->TransformationMatrix(robotInfo->toolID, rml::FrameID::WorldFrame).block<3,3>(0,0);
    for (auto nJoint = 0; nJoint < 6; nJoint++) {
        //std::cerr << "[MoveJointsPos::Execute] Tjoint" << (nJoint + 1) << " = " << std::endl << robotModel->TransformationMatrix("Left" +
            //rml::FrameID::Joint + std::to_string(nJoint)) << std::endl;
    }
   // std::cerr << "[MoveJointsPos::Execute] wTtool = " << std::endl << robotModel->TransformationMatrix(robotInfo->toolID, rml::FrameID::WorldFrame) << std::endl;
  //  std::cerr << "[MoveJointsPos::Execute] RPYtool = " << rotTool.ToEulerRPY() << std::endl;

    jointsPositionTask_->Reference() = robotInfo->jointsGoal;

    Eigen::VectorXd error = robotInfo->jointsGoal - armModel->JointsPosition();
    
    std::cerr << "[MoveJointsPos::Execute] Our joint pos = " << armModel->JointsPosition().transpose() << std::endl;
    
    jointsPositionTask_->Update();
    std::cerr << "[MoveJointsPos::Execute] Joint pos error = " << error.transpose() << std::endl;
    std::cerr << "[MoveJointsPos::Execute] Joint goal = " << robotInfo->jointsGoal.transpose() << std::endl;
    std::cout << "[MoveJointsPos::Execute] Elapsed time is " << moveTimer_.Elapsed() << " but max is " << conf->cartesianMoveTimeout << std::endl;
    if (error.cwiseAbs().maxCoeff() < 0.01) { //conf->angularErrorThreshold) {

        std::cout << "[KCL] Joint Position Reached." << std::endl;
        fsm_->SetNextState(appleRobot::ID::states::idle);
        ok_ = true;
    } else if (moveTimer_.Elapsed() > conf->cartesianMoveTimeout) {
        std::cout << "[KCL]: Joint Position Move Timeout Reached." << std::endl;
        fsm_->SetNextState(appleRobot::ID::states::idle);
        ok_ = false;
    }
     //std::cerr << "[MoveJointsPos::Execute()] Err is " <<  error.cwiseAbs().maxCoeff() << " > " << conf->angularErrorThreshold << std::endl;

    return fsm::ok;
}

}
