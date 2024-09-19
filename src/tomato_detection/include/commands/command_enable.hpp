#ifndef UNDROID_CTRL_COMMANDENABLE_HPP
#define UNDROID_CTRL_COMMANDENABLE_HPP

#include "commands/generic_command.hpp"
#include "states/state_disabled.hpp"
#include "states/state_idle.hpp"

namespace commands {

class Enable : public GenericCommand {

private:
    std::shared_ptr<states::Idle> stateIdle_;
    ros::Publisher driverCmdPub_;

public:
    Enable();
    virtual ~Enable() override;
    virtual fsm::retval Execute() override;

    void SetState(std::shared_ptr<states::GenericState> state) override;
    void SetNodeHandle(std::shared_ptr<ros::NodeHandle> nh) override;

};
}

#endif // UNDROID_CTRL_COMMANDENABLE_HPP
