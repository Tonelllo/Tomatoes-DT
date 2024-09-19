#include "states/state_parking.hpp"



namespace states {

Parking::Parking() { }

Parking::~Parking() { }

bool Parking::ConfigureStateFromFile(libconfig::Config& confObj)
{
    (void) confObj;
    //const libconfig::Setting& root = confObj.getRoot();
    //const libconfig::Setting& states = root["states"];
    //
    //const libconfig::Setting& state = states.lookup(appleRobot::ID::states::parking);
    //parkJointsPos_.resize(10);
    //if (!ctb::GetParamVector(state, parkJointsPos_, "parkJointsPos"))
    //    return false;

    return true;
}


fsm::retval Parking::OnEntry()
{

    //Eigen::Map<Eigen::VectorXd> jointsGoal(parkJointsPos_.data(), robotModel->Dof());
    //robotInfo->jointsGoal = ;

    jointsPositionTask_ = std::dynamic_pointer_cast<ikcl::JointsPosition>(tasksMap.find(appleRobot::ID::Tasks::JointsPosition)->second.task);

    if (actionManager->SetAction(appleRobot::ID::Actions::ParkUnpark, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }
}

fsm::retval Parking::Execute()
{
    jointsPositionTask_->Reference() = robotInfo->parkedJointsPos;

    if (robotInfo->arm_state_id == appleRobot::ID::GTArmState::PARKED) {
        fsm_->SetNextState(appleRobot::ID::states::parked);
    }

    //Eigen::VectorXd error = armModel->JointsPosition() - robotInfo->parkedJointsPos;
    //
    //if (error.cwiseAbs().maxCoeff() < conf->angularErrorThreshold) {
    //
    //    std::cout << "[KCL] Joint Position Reached." << std::endl;
    //    fsm_->SetNextState(appleRobot::ID::states::disabled);
    //}

    return fsm::ok;
}

}
