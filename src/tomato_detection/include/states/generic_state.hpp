#ifndef UNDROID_CTRL_GENERIC_STATE_HPP
#define UNDROID_CTRL_GENERIC_STATE_HPP

#include "agri_control/ctrl_defines.hpp"
#include "apple_robot_defines.hpp"
#include "agri_control/futils.hpp"

#include <fsm/fsm.h>
#include <ikcl/ikcl.h>
#include <libconfig.h++>

#include "tomato_detection/ControlCommand.h"


namespace states {

class GenericState : public fsm::BaseState {

public:
    std::shared_ptr<RobotInfo> robotInfo;
    std::shared_ptr<tpik::ActionManager> actionManager;
    std::shared_ptr<rml::RobotModel> robotModel;
    std::unordered_map<std::string, TasksInfo> tasksMap;
    std::shared_ptr<KCLConfiguration> conf;

    GenericState();
    virtual ~GenericState(void);

    virtual bool ConfigureStateFromFile(libconfig::Config& confObj) = 0;
};
}

#endif // UNDROID_CTRL_GENERIC_STATE_HPP
