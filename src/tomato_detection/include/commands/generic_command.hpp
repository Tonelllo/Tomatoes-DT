#ifndef UNDROID_CTRL_GENERICCOMMAND_HPP
#define UNDROID_CTRL_GENERICCOMMAND_HPP

#include <ros/ros.h>
#include <fsm/fsm.h>
#include "states/generic_state.hpp"


namespace commands {

class GenericCommand : public fsm::BaseCommand {
protected:
    std::shared_ptr<ros::NodeHandle> nh;
public:

    std::shared_ptr<RobotInfo> robotInfo;


    GenericCommand();
    virtual ~GenericCommand();

    virtual void SetState(std::shared_ptr<states::GenericState> state) = 0;
    virtual void SetNodeHandle(std::shared_ptr<ros::NodeHandle> nh) = 0;
};

}
#endif // UNDROID_CTRL_GENERICCOMMAND_HPP
