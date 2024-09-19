#ifndef CONTROL_CONFIGURATION_H
#define CONTROL_CONFIGURATION_H

#include <ikcl/ikcl.h>
#include "ctrl_defines.hpp"
//#include "states/generic_state.hpp"

bool ConfigureTasksFromFile(std::unordered_map<std::string, TasksInfo>& tasksMap, libconfig::Config& confObj);
bool ConfigurePriorityLevelsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, TasksInfo>& tasksMap, libconfig::Config& confObj);
bool ConfigureActionsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, libconfig::Config& confObj);
//bool ConfigureSatesFromFile(std::unordered_map<std::string, std::shared_ptr<states::GenericState>> statesMap, libconfig::Config& confObj);
#endif // CONTROL_CONFIGURATION_H
