#ifndef UNDROID_CTRL_STATEDISABLED_HPP
#define UNDROID_CTRL_STATEDISABLED_HPP

#include "states/generic_state.hpp"


namespace states {

    class Disabled : public GenericState {

    public:
        Disabled();
        ~Disabled() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;

    };
}


#endif // UNDROID_CTRL_STATEDISABLED_HPP
