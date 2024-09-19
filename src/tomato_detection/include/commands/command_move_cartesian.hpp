#ifndef UNDROID_CTRL_COMMANDMOVECARTESIAN_HPP
#define UNDROID_CTRL_COMMANDMOVECARTESIAN_HPP

#include "commands/generic_command.hpp"
#include "states/state_move_cartesian.hpp"


namespace commands {

class MoveCartesian : public GenericCommand {

private:
    std::shared_ptr<states::MoveCartesian> stateMoveCartesian_;
    boost::array<double, 3> targetXYZ_;
    boost::array<double, 3> targetRPY_;
    std::string moveType_;
    int frameType_;

public:
    MoveCartesian();
    virtual ~MoveCartesian() override;
    virtual fsm::retval Execute() override;

    void SetState(std::shared_ptr<states::GenericState> state) override;
    void SetNodeHandle(std::shared_ptr<ros::NodeHandle> nh) override;

    void SetCartesianGoal(boost::array<double, 3> target_xyz, boost::array<double, 3> target_rpy, const std::string &move_type, int frame_type);
};
}

#endif // UNDROID_CTRL_COMMANDMOVECARTESIAN_HPP
