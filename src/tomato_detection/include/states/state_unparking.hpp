#ifndef UNDROID_CTRL_STATEUNPARK_HPP
#define UNDROID_CTRL_STATEUNPARK_HPP

#include "states/generic_state.hpp"


namespace states {

class Unparking : public GenericState {

    std::shared_ptr<ikcl::JointsPosition> jointsPositionTask_;

public:
    Unparking();
    ~Unparking() override;
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;

    std::shared_ptr<rml::ArmModel> armModel;

    bool ConfigureStateFromFile(libconfig::Config& confObj) override;

};
}


#endif // UNDROID_CTRL_STATEUNPARK_HPP
