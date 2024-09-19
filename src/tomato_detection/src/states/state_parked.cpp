#include "states/state_parked.hpp"



namespace states {

Parked::Parked() { }

Parked::~Parked() { }

bool Parked::ConfigureStateFromFile(libconfig::Config& confObj)
{
    (void) confObj;
    //const libconfig::Setting& root = confObj.getRoot();
    //const libconfig::Setting& states = root["states"];
    //
    //const libconfig::Setting& state = states.lookup(undroid::ID::states::Parked);
    //parkJointsPos_.resize(10);
    //if (!ctb::GetParamVector(state, parkJointsPos_, "parkJointsPos"))
    //    return false;

    return true;
}


fsm::retval Parked::OnEntry()
{

    //Eigen::Map<Eigen::VectorXd> jointsGoal(parkJointsPos_.data(), robotModel->Dof());
    //robotInfo->jointsGoal = ;

    //jointsPositionTask_ = std::dynamic_pointer_cast<ikcl::JointsPosition>(tasksMap.find(undroid::ID::Tasks::JointsPosition)->second.task);

    //if (actionManager->SetAction(undroid::ID::Actions::ParkUnpark, true)) {
    return fsm::ok;
    //} else {
    //    return fsm::fail;
    //}
}

fsm::retval Parked::Execute()
{
    if (robotInfo->requestedUnpark) {
        std::cout << "[KCL] UNPARKING" << std::endl;
        robotInfo->requestedUnpark = false;
        return fsm_->ExecuteCommand(appleRobot::ID::commands::unpark);
    }

    return fsm::ok;
}

}
