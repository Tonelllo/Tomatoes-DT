#ifndef UNDROID_CTRL_STATEIDLE_HPP
#define UNDROID_CTRL_STATEIDLE_HPP

#include "states/generic_state.hpp"


namespace states {

    class Idle : public GenericState {

    public:
        Idle();
        ~Idle() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}


#endif // UNDROID_CTRL_STATEIDLE_HPP
