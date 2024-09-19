#ifndef UNDROID_CTRL_COMMANDPARK_HPP
#define UNDROID_CTRL_COMMANDPARK_HPP

#include "commands/generic_command.hpp"
#include "states/state_parking.hpp"


namespace commands {

class Park : public GenericCommand {

private:
    std::shared_ptr<states::Parking> stateParking_;
    ros::Publisher driverCmdPub_;

public:
    Park();
    virtual ~Park() override;
    virtual fsm::retval Execute() override;

    void SetState(std::shared_ptr<states::GenericState> state) override;
    void SetNodeHandle(std::shared_ptr<ros::NodeHandle> nh) override;

};
}

#endif // UNDROID_CTRL_COMMANDPARK_HPP
