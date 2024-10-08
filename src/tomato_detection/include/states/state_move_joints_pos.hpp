#ifndef UNDROID_CTRL_STATEMOVEJOINTSPOS_HPP
#define UNDROID_CTRL_STATEMOVEJOINTSPOS_HPP

#include "states/generic_state.hpp"


namespace states {

class MoveJointsPos : public GenericState {


    std::shared_ptr<ikcl::JointsPosition> jointsPositionTask_;
    futils::Timer moveTimer_;

public:
    MoveJointsPos();
    ~MoveJointsPos() override;
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
    std::shared_ptr<rml::ArmModel> armModel;

    bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    bool SetJointsGoal(std::vector<double> &joints_pos, std::string &move_type);
    bool SetSingleJointGoal(std::vector<double> &joints_pos, int i, std::string &move_type);
    bool ok_;

    bool enableObstacleAvoidance;
    
    size_t cnt;

};
}


#endif // UNDROID_CTRL_STATEMOVEJOINTSPOS_HPP
