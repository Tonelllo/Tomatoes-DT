#include "commands/command_enable.hpp"

#include "tomato_detection/DriverCommand.h"

namespace commands {

Enable::Enable() {

}

Enable::~Enable() {}

fsm::retval Enable::Execute()
{
    robotInfo->requestedEnable = true;
    tomato_detection::DriverCommand driverCmd;
    driverCmd.command_type = appleRobot::ID::GTManipulatorCommand::ARMCMD;
    driverCmd.command_id = appleRobot::ID::GTArmCommand::ENABLE;
    driverCmdPub_.publish(driverCmd);

    //ROS_INFO("Enabling Manipulator");
    return fsm::ok;

}

void Enable::SetState(std::shared_ptr<states::GenericState> state)
{
    stateIdle_ = std::dynamic_pointer_cast<states::Idle>(state);
}

void Enable::SetNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
    nh = nodeHandle;
    driverCmdPub_ = nh->advertise<tomato_detection::DriverCommand>(appleRobot::topicnames::driver_cmd, 10);
}

}

