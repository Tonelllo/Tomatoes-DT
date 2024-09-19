#include "states/state_move_joints_vel.hpp"



namespace states {

MoveJointsVel::MoveJointsVel() { }

MoveJointsVel::~MoveJointsVel() { }

bool MoveJointsVel::ConfigureStateFromFile(libconfig::Config& confObj)
{
    (void) confObj;
    //const libconfig::Setting& root = confObj.getRoot();
    //const libconfig::Setting& states = root["states"];

    /*const libconfig::Setting& state = states.lookup(appleRobot::ID::states::halt);
    if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
        return false;
    if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
        return false;*/

    return true;
}

bool MoveJointsVel::SetJointsGoal(std::vector<double> &joints_vel)
{
    if (joints_vel.size() == armModel->NumJoints()) {
        Eigen::Map<Eigen::VectorXd> jointsGoal(joints_vel.data(), robotModel->Dof());
        robotInfo->jointsGoal = jointsGoal;
        timeoutTimer.Start();
        return true;
    } else {
        return false;
    }
}

bool MoveJointsVel::SetSingleJointGoal(std::vector<double> &joint_vel, int i)
{

    if (joint_vel.size() == 1) {
        robotInfo->jointsGoal = Eigen::VectorXd::Zero(armModel->NumJoints());
        robotInfo->jointsGoal(i) = joint_vel.at(0);
        timeoutTimer.Start();
        return true;
    } else {
        return false;
    }

}

fsm::retval MoveJointsVel::OnEntry()
{
    jointsVelocityTask_ = std::dynamic_pointer_cast<ikcl::JointsVelocity>(tasksMap.find(appleRobot::ID::Tasks::JointsVelocity)->second.task);

    if (actionManager->SetAction(appleRobot::ID::Actions::MoveJointsVelocity, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }
}

fsm::retval MoveJointsVel::Execute()
{
    jointsVelocityTask_->SetReferenceRate(robotInfo->jointsGoal);
    //std::cout << "robotInfo->jointsGoal: " << robotInfo->jointsGoal.transpose() << std::endl;

    if (timeoutTimer.Elapsed() > 1.0) {
        timeoutTimer.Stop();
        std::cout << "[KCL]: 1s Timeout Reached." << std::endl;

        fsm_->SetNextState(appleRobot::ID::states::idle);
        robotInfo->jointsGoal.setZero();
    }

    return fsm::ok;
}

}
