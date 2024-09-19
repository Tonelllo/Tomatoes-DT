#ifndef UNDROID_CTRL_COMMANDMOVEJOINTSVEL_HPP
#define UNDROID_CTRL_COMMANDMOVEJOINTSVEL_HPP

#include "commands/generic_command.hpp"
#include "states/state_move_joints_vel.hpp"


namespace commands {

class MoveJointsVel : public GenericCommand {

private:
    std::shared_ptr<states::MoveJointsVel> stateMoveJointsVel_;
    std::vector<double> jointsVel_;
    int jointIndex_;
    bool singleJointMove;

public:
    MoveJointsVel();
    virtual ~MoveJointsVel() override;
    virtual fsm::retval Execute() override;

    void SetState(std::shared_ptr<states::GenericState> state) override;
    void SetNodeHandle(std::shared_ptr<ros::NodeHandle> nh) override;

    void SetJointsGoal(std::vector<double> &joints_vel);
    void SetSingleJointGoal(std::vector<double> &joints_vel, int i);
};
}

#endif // UNDROID_CTRL_COMMANDMOVEJOINTSVEL_HPP
