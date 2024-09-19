#include <libconfig.h++>
#include "agri_control/control_configuration.hpp"

bool ConfigureTasksFromFile(std::unordered_map<std::string, TasksInfo> &tasksMap, libconfig::Config& confObj)
{
    for (auto& map : tasksMap) {
        if (!map.second.task->ConfigFromFile(confObj)) {
            std::cerr << "Failed to configure task " << map.first << " from file" << std::endl;
            return false;
        }

        std::cout << map.second.task->ID() << std::endl;
        std::cout << "enable: " << map.second.task->Enabled() << std::endl;
        std::cout << "- - -" << std::endl;
    }

    return true;
}

bool ConfigurePriorityLevelsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, TasksInfo>& tasksMap, libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& priorityLevels = root["priorityLevels"];

    std::vector<std::string> hierarchy;

    for (int i = 0; i < priorityLevels.getLength(); ++i) {

        const libconfig::Setting& priorityLevel = priorityLevels[i];
        std::string PLID;

        if (!ctb::GetParam(priorityLevel, PLID, "name"))
            return false;

        hierarchy.push_back(PLID);

        //configure regularization data
        rml::RegularizationData regularizationData;

        if (!ctb::GetParam(priorityLevel, regularizationData.params.lambda, "lambda"))
            return false;
        if (!ctb::GetParam(priorityLevel, regularizationData.params.threshold, "threshold"))
            return false;

        actionManager->AddPriorityLevelWithRegularization(PLID, regularizationData);

        std::cout << "Added: Priority Level " << PLID << " with tasks:" << std::endl;

        for (auto task : tasksMap) {
            std::cerr << "task " << task.first << " --> " << std::endl;
        }

        //add tasks to PL
        const libconfig::Setting& tasks = priorityLevel["tasks"];
        std::cout << "[ConfigurePriorityLevelsFromFile] Point 1" << std::endl;
        for (int i = 0; i < tasks.getLength(); ++i) {
            std::cout << "[ConfigurePriorityLevelsFromFile] " << i << " --> Point 0" << std::endl;
            libconfig::Setting& task = tasks[i];
            std::cout << "[ConfigurePriorityLevelsFromFile] " << i << " --> Point 1" << std::endl;

            std::cout << "  - " << task.c_str() << std::endl;
            std::cout << "[ConfigurePriorityLevelsFromFile] " << i << " --> Point 2" << std::endl;

            std::unordered_map<std::string, TasksInfo>::iterator it = tasksMap.find(task.c_str());
            std::cout << "[ConfigurePriorityLevelsFromFile] " << i << " --> Point 3" << std::endl;

            //std::cout << "[ConfigurePriorityLevelsFromFile] *it = " << *it << std::endl;
            std::cout << "[ConfigurePriorityLevelsFromFile] it->second.task = " << it->second.task << std::endl;
            std::cout << "[ConfigurePriorityLevelsFromFile] PLID = " << PLID << std::endl;
            actionManager->AddTaskToPriorityLevel(it->second.task, PLID);
            std::cout << "[ConfigurePriorityLevelsFromFile] " << i << " --> Point 4" << std::endl;
        }
        std::cout << "[ConfigurePriorityLevelsFromFile] Priority Level Ok" << std::endl;
    }
    //Set the hierarchy
    actionManager->SetUnifiedHierarchy(hierarchy);



    return true;
}

bool ConfigureActionsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& actions = root["actions"];

    std::vector<std::string> actionPL;

    for (int i = 0; i < actions.getLength(); ++i) {

        const libconfig::Setting& action = actions[i];
        std::string actionID;
        if (!ctb::GetParam(action, actionID, "name"))
            return false;

        std::vector<std::string> actionPL;

        const libconfig::Setting& priorityLevels = action["levels"];
        for (int i = 0; i < priorityLevels.getLength(); ++i) {

            libconfig::Setting& priorityLevel = priorityLevels[i];

            actionPL.push_back(priorityLevel.c_str());
        }

        try {
            actionManager->AddAction(actionID, actionPL);
            std::cout << "Added: " << actionID << " action with PLs:" << std::endl;
            for (auto& pl : actionPL)
                std::cout << "  - " << pl << std::endl;

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "Configuration Action Manager Exception:" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }


    return true;
}

/*bool ConfigureSatesFromFile(std::unordered_map<std::string, std::shared_ptr<states::GenericState> > statesMap, libconfig::Config& confObj)
{
    for (auto& mapLine : statesMap) {
        if (!mapLine.second->ConfigureStateFromFile(confObj)) {
            std::cerr << "Failed to configure from file: " << mapLine.first << std::endl;
            return false;
        }
    }
    return true;
}
*/