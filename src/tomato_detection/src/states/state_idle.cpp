#include "states/state_idle.hpp"


namespace states {

Idle::Idle() { }

Idle::~Idle() { }

bool Idle::ConfigureStateFromFile(libconfig::Config& confObj)
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

fsm::retval Idle::OnEntry()
{
    //std::cout << tc::mag << "State: " << fsm_->GetNextStateName() << "." << tc::none << std::endl;

    if (actionManager->SetAction(appleRobot::ID::Actions::Idle, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }
}

fsm::retval Idle::Execute()
{


    //std::cout << "STATE Idle" << std::endl;
    return fsm::ok;
}

}
