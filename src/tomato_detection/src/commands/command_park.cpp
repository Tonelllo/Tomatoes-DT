#include "commands/command_park.hpp"

#include "tomato_detection/DriverCommand.h"

namespace commands {

Park::Park() {}

Park::~Park() {}

fsm::retval Park::Execute()
{
    tomato_detection::DriverCommand driverCmd;
    driverCmd.command_type = appleRobot::ID::GTManipulatorCommand::ARMCMD;
    driverCmd.command_id = appleRobot::ID::GTArmCommand::PARK;
    driverCmdPub_.publish(driverCmd);


    fsm_->SetNextState(appleRobot::ID::states::parking);

    return fsm::ok;

}

void Park::SetState(std::shared_ptr<states::GenericState> state)
{
    stateParking_ = std::dynamic_pointer_cast<states::Parking>(state);
}

void Park::SetNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
    nh = nodeHandle;
    driverCmdPub_ = nh->advertise<tomato_detection::DriverCommand>(appleRobot::topicnames::driver_cmd, 10);

}

}

