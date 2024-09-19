#ifndef UNDROID_CTRL_STATEParking_HPP
#define UNDROID_CTRL_STATEParking_HPP

#include "states/generic_state.hpp"


namespace states {

class Parking : public GenericState {


    std::shared_ptr<ikcl::JointsPosition> jointsPositionTask_;

public:
    Parking();
    ~Parking() override;
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;

    std::shared_ptr<rml::ArmModel> armModel;

    bool ConfigureStateFromFile(libconfig::Config& confObj) override;
};
}


#endif // UNDROID_CTRL_STATEParking_HPP
