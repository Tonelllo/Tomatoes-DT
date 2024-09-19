#include "commands/command_disable.hpp"

#include "tomato_detection/DriverCommand.h"

namespace commands {

Disable::Disable() {}

Disable::~Disable() {}

fsm::retval Disable::Execute()
{
    tomato_detection::DriverCommand driverCmd;
    driverCmd.command_type = appleRobot::ID::GTManipulatorCommand::ARMCMD;
    driverCmd.command_id = appleRobot::ID::GTArmCommand::DISABLE;
    driverCmdPub_.publish(driverCmd);

    return fsm_->SetNextState(appleRobot::ID::states::disabled);
}

void Disable::SetState(std::shared_ptr<states::GenericState> state)
{
    stateDisabled_ = std::dynamic_pointer_cast<states::Disabled>(state);
}

void Disable::SetNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
    nh = nodeHandle;
    driverCmdPub_ = nh->advertise<tomato_detection::DriverCommand>(appleRobot::topicnames::driver_cmd, 10);

}
}

