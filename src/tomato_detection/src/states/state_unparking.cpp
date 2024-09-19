#include "states/state_unparking.hpp"



namespace states {

Unparking::Unparking() { }

Unparking::~Unparking() { }

bool Unparking::ConfigureStateFromFile(libconfig::Config& confObj)
{
    (void) confObj;
//    const libconfig::Setting& root = confObj.getRoot();
//    const libconfig::Setting& states = root["states"];
//
//    const libconfig::Setting& state = states.lookup(appleRobot::ID::states::unparking);
//    unparkJointsPos_.resize(10);
//    if (!ctb::GetParamVector(state, unparkJointsPos_, "unparkJointsPos"))
//        return false;
//
    return true;
}


fsm::retval Unparking::OnEntry()
{
    //Eigen::Map<Eigen::VectorXd> jointsGoal(UnparkingJointsPos_.data(), robotModel->Dof());


    //unparkJointsPos_.resize(robotModel->Dof());

    jointsPositionTask_ = std::dynamic_pointer_cast<ikcl::JointsPosition>(tasksMap.find(appleRobot::ID::Tasks::JointsPosition)->second.task);

    std::cout << "unparkedJointsPos: " << robotInfo->unparkedJointsPos.transpose() << std::endl;

    if (actionManager->SetAction(appleRobot::ID::Actions::ParkUnpark, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }
}

fsm::retval Unparking::Execute()
{
    jointsPositionTask_->Reference() = robotInfo->unparkedJointsPos;

    if (robotInfo->arm_state_id == appleRobot::ID::GTArmState::READY) {
        fsm_->SetNextState(appleRobot::ID::states::idle);
    }

    //Eigen::VectorXd error = armModel->JointsPosition() - robotInfo->unparkedJointsPos;
    //
    //if (error.cwiseAbs().maxCoeff() < conf->angularErrorThreshold) {
    //
    //    std::cout << "[KCL] Unparked." << std::endl;
    //    fsm_->SetNextState(appleRobot::ID::states::idle);
    //}

    return fsm::ok;
}

}
