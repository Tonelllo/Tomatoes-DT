#include "commands/command_move_joints_vel.hpp"


namespace commands {

MoveJointsVel::MoveJointsVel() {}

MoveJointsVel::~MoveJointsVel() {}

void MoveJointsVel::SetState(std::shared_ptr<states::GenericState> state)
{
    stateMoveJointsVel_ = std::dynamic_pointer_cast<states::MoveJointsVel>(state);
}

void MoveJointsVel::SetNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
    nh = nodeHandle;
}

fsm::retval MoveJointsVel::Execute()
{
    auto ret = fsm::ok;

    if (singleJointMove) {
        if(!stateMoveJointsVel_->SetSingleJointGoal(jointsVel_, jointIndex_)) ret = fsm::fail;
    } else {
        if(!stateMoveJointsVel_->SetJointsGoal(jointsVel_)) ret = fsm::fail;
    }

    if(ret == fsm::ok) {
        ret = fsm_->SetNextState(appleRobot::ID::states::move_joints_vel);
        std::cout << "[MoveJointsVel] Ref: " << futils::STLVectorToString(jointsVel_, ',') << std::endl;
    } else {
        ROS_ERROR("Error in Setting Joint Goal (check command parameters)");
    }

    return ret;
}



void commands::MoveJointsVel::SetJointsGoal(std::vector<double> &joints_vel)
{
    jointsVel_ = joints_vel;
    singleJointMove = false;
}

void MoveJointsVel::SetSingleJointGoal(std::vector<double> &joints_vel, int i)
{
    jointsVel_ = joints_vel;
    jointIndex_ = i;
    singleJointMove = true;
}


}

