#include "commands/command_stop.hpp"

#include "tomato_detection/DriverCommand.h"

namespace commands {

Stop::Stop() {}

Stop::~Stop() {}

fsm::retval Stop::Execute()
{
    tomato_detection::DriverCommand driverCmd;
    driverCmd.command_type = appleRobot::ID::GTManipulatorCommand::ARMCMD;
    driverCmd.command_id = appleRobot::ID::GTArmCommand::STOP;
    driverCmdPub_.publish(driverCmd);

    return fsm_->SetNextState(appleRobot::ID::states::idle);
}

void Stop::SetState(std::shared_ptr<states::GenericState> state)
{
    stateIdle_ = std::dynamic_pointer_cast<states::Idle>(state);
}

void Stop::SetNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
    nh = nodeHandle;
        driverCmdPub_ = nh->advertise<tomato_detection::DriverCommand>(appleRobot::topicnames::driver_cmd, 10);
}

}

