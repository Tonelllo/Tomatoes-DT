#include "states/state_disabled.hpp"
#include "apple_robot_defines.hpp"


namespace states {

Disabled::Disabled() { }

Disabled::~Disabled() { }

bool Disabled::ConfigureStateFromFile(libconfig::Config& confObj)
{
    (void) confObj;
    //const libconfig::Setting& root = confObj.getRoot();
    //const libconfig::Setting& states = root["states"];

    /*const libconfig::Setting& state = states.lookup(undroid::ID::states::halt);
    if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
        return false;
    if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
        return false;*/

    return true;
}

fsm::retval Disabled::OnEntry()
{
    //std::cout << tc::mag << "State: " << fsm_->GetNextStateName() << "." << tc::none << std::endl;
    return fsm::ok;

}

fsm::retval Disabled::Execute()
{
    //std::cout << "STATE Disabled" << std::endl;

    if (robotInfo->arm_state_id == appleRobot::ID::GTArmState::DISABLED)
    {
        // The robot will be switching to one of the two states below
    }
    else if (robotInfo->arm_state_id == appleRobot::ID::GTArmState::PARKED)
    {
        return fsm_->SetNextState(appleRobot::ID::states::parked);


    } else if (robotInfo->arm_state_id == appleRobot::ID::GTArmState::READY)
    {
        std::cout << "[KCL] Robot ready" << std::endl;
        return fsm_->SetNextState(appleRobot::ID::states::idle);
    }



    return fsm::ok;
}

}
