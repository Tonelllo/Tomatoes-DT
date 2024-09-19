#include "commands/command_unpark.hpp"

#include "tomato_detection/DriverCommand.h"

namespace commands {

Unpark::Unpark() {}

Unpark::~Unpark() {}

fsm::retval Unpark::Execute()
{
    tomato_detection::DriverCommand driverCmd;
    driverCmd.command_type = appleRobot::ID::GTManipulatorCommand::ARMCMD;
    driverCmd.command_id = appleRobot::ID::GTArmCommand::EXIT;
    driverCmdPub_.publish(driverCmd);

    //ROS_INFO("Unparking Manipulator");
    return fsm_->SetNextState(appleRobot::ID::states::unparking);;

}

void Unpark::SetState(std::shared_ptr<states::GenericState> state)
{
    stateUnparking_ = std::dynamic_pointer_cast<states::Unparking>(state);
}

void Unpark::SetNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
    nh = nodeHandle;
    driverCmdPub_ = nh->advertise<tomato_detection::DriverCommand>(appleRobot::topicnames::driver_cmd, 10);}


}

