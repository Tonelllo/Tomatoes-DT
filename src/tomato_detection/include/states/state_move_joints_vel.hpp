#ifndef UNDROID_CTRL_STATEMOVEJOINTSVEL_HPP
#define UNDROID_CTRL_STATEMOVEJOINTSVEL_HPP

#include "states/generic_state.hpp"


namespace states {

class MoveJointsVel : public GenericState {


    std::shared_ptr<ikcl::JointsVelocity> jointsVelocityTask_;
    futils::Timer timeoutTimer;

public:
    MoveJointsVel();
    ~MoveJointsVel() override;
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;

    std::shared_ptr<rml::ArmModel> armModel;

    bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    bool SetJointsGoal(std::vector<double> &joints_vel);
    bool SetSingleJointGoal(std::vector<double> &joints_vel, int i);
};
}


#endif // UNDROID_CTRL_STATEMOVEJOINTSVEL_HPP
