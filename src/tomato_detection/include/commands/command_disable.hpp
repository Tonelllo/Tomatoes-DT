#ifndef UNDROID_CTRL_COMMANDDISABLE_HPP
#define UNDROID_CTRL_COMMANDDISABLE_HPP

#include "commands/generic_command.hpp"
#include "states/state_disabled.hpp"


namespace commands {

class Disable : public GenericCommand {

private:
    std::shared_ptr<states::Disabled> stateDisabled_;
    ros::Publisher driverCmdPub_;

public:
    Disable();
    virtual ~Disable() override;
    virtual fsm::retval Execute() override;

    void SetState(std::shared_ptr<states::GenericState> state) override;
    void SetNodeHandle(std::shared_ptr<ros::NodeHandle> nh) override;
};
}

#endif // UNDROID_CTRL_COMMANDDISABLE_HPP
