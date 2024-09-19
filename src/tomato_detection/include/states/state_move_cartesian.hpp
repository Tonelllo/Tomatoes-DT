#ifndef UNDROID_CTRL_STATEMOVECARTESIAN_HPP
#define UNDROID_CTRL_STATEMOVECARTESIAN_HPP

#include "states/generic_state.hpp"


namespace states {

class MoveCartesian : public GenericState {

    std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;
    std::shared_ptr<ikcl::CartesianOrientation> cartesianOrientationTask_;
    futils::Timer moveTimer_;

public:
    MoveCartesian();
    ~MoveCartesian() override;
    fsm::retval OnEntry() override;
    fsm::retval Execute() override;
    fsm::retval OnExit() override;
    bool SetCartesianGoal(boost::array<double, 3> target_xyz, boost::array<double, 3> target_rpy,
                          const std::string &move_type, int frame_type);

    bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    bool ok_;

};
}


#endif // UNDROID_CTRL_STATEMOVEJOINTS_HPP
