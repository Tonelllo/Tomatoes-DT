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
    cnt = 0;

    jointsPositionTask_ = std::dynamic_pointer_cast<ikcl::JointsPosition>(tasksMap.find(appleRobot::ID::Tasks::JointsPosition)->second.task);
    ok_ = true;

    if (!enableObstacleAvoidance) {
        //auto eaf = tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction();
       // tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction() = eaf.setZero();
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
    //auto eaf = tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction();
    //tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction() = eaf.setIdentity();
    //std::cerr << "[MoveJointsPos::OnEntry] tmp eaf after oneing = " << tasksMap[appleRobot::ID::Tasks::ObstacleAvoidance].task->ExternalActivationFunction() << std::endl;
    return fsm::ok;
}


fsm::retval MoveJointsPos::Execute()
{
    cnt++;
    auto enableDbgPrnt = cnt % 40 == 0;

    jointsPositionTask_->Reference() = robotInfo->jointsGoal;

    Eigen::VectorXd error = robotInfo->jointsGoal - armModel->JointsPosition();
    jointsPositionTask_->Update();

    if (enableDbgPrnt) {
        std::cerr << tc::yellow << "[MoveJointsPos::Execute] TASK INFO Start..." << tc::none << std::endl;
        std::cerr << "robotModel->TransformationMatrix(ctrlInfo->toolID): " << std::endl << robotModel->TransformationMatrix(robotInfo->toolID) <<  std::endl;
        std::cerr << tc::magL << "[MoveCartesian::Execute] wTtool = " << std::endl << robotModel->TransformationMatrix(rml::FrameID::WorldFrame, robotInfo->toolID) << std::endl;
        std::cerr << tc::magL << "[MoveJointsPos::Execute] Current joint pos = " << armModel->JointsPosition().transpose() << tc::none << std::endl;
        std::cerr << tc::magL << "[MoveJointsPos::Execute] Current joint pos = " << armModel->JointsPosition().transpose() << tc::none << std::endl;
        std::cerr << tc::magL << "[MoveJointsPos::Execute] Joint goal = " << robotInfo->jointsGoal.transpose() << tc::none << std::endl;
        std::cerr << tc::yellow << "[MoveJointsPos::Execute] Joint pos error = " << error.transpose();
        std::cerr << " (abs val is " << error.cwiseAbs().maxCoeff() << ")" << tc::none << std::endl;
        auto clrTime = tc::bluL;
        if (conf->cartesianMoveTimeout - moveTimer_.Elapsed() < 5) clrTime = tc::redL;
        else if (conf->cartesianMoveTimeout - moveTimer_.Elapsed() < 10) clrTime = tc::yellow;
        std::cout << clrTime << "[MoveJointsPos::Execute] Elapsed time is " << moveTimer_.Elapsed() << " out of " <<
            conf->cartesianMoveTimeout << tc::none << std::endl;
        auto iaf = tasksMap[appleRobot::ID::Tasks::JointsLimit].task->InternalActivationFunction().diagonal().transpose();
        std::cerr << "iaf = " << iaf << std::endl;
        auto eaf = tasksMap[appleRobot::ID::Tasks::JointsLimit].task->ExternalActivationFunction().diagonal().transpose();
        std::cerr << "eaf = " << eaf << std::endl;
    }
    if (error.cwiseAbs().maxCoeff() < conf->angularErrorThreshold) {

        std::cout << tc::greenL << "[KCL] Joint Position Reached." << tc::none << std::endl;
       // fsm_->SetNextState(appleRobot::ID::states::idle);
        ok_ = true;
    } else if (moveTimer_.Elapsed() > conf->cartesianMoveTimeout) {
        std::cout << tc::redL << "[KCL]: Joint Position Move Timeout Reached." << tc::none << std::endl;
       // fsm_->SetNextState(appleRobot::ID::states::idle);
        ok_ = false;
    }
    if (enableDbgPrnt) std::cerr << tc::yellow << "[MoveJointsPos::Execute] TASK INFO Finish!" << tc::none << std::endl;

    return fsm::ok;
}

}
