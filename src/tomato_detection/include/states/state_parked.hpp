#ifndef UNDROID_CTRL_STATEParked_HPP
#define UNDROID_CTRL_STATEParked_HPP

#include "states/generic_state.hpp"


namespace states {

class Parked : public GenericState {


    //std::shared_ptr<ikcl::JointsPosition> jointsPositionTask_;

public:
    Parked();
    ~Parked() override;
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;

    std::shared_ptr<rml::ArmModel> armModel;

    bool ConfigureStateFromFile(libconfig::Config& confObj) override;
};
}


#endif // UNDROID_CTRL_STATEParked_HPP
