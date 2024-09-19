#ifndef UNDROID_CTRL_COMMANDUNPARK_HPP
#define UNDROID_CTRL_COMMANDUNPARK_HPP

#include "commands/generic_command.hpp"
#include "states/state_unparking.hpp"


namespace commands {

class Unpark : public GenericCommand {

private:
    std::shared_ptr<states::Unparking> stateUnparking_;
    ros::Publisher driverCmdPub_;

public:
    Unpark();
    virtual ~Unpark() override;
    virtual fsm::retval Execute() override;

    void SetState(std::shared_ptr<states::GenericState> state) override;
    void SetNodeHandle(std::shared_ptr<ros::NodeHandle> nh) override;

};
}

#endif // UNDROID_CTRL_COMMANDUNPARK_HPP
