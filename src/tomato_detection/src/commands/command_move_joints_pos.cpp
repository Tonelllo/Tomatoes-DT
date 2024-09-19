#include "commands/command_move_joints_pos.hpp"


namespace commands {

MoveJointsPos::MoveJointsPos() {}

MoveJointsPos::~MoveJointsPos() {}

fsm::retval MoveJointsPos::Execute()
{
    auto ret = fsm::ok;

    if (singleJointMove) {
        if(!stateMoveJointsPos_->SetSingleJointGoal(jointsPos_, jointIndex_, moveType_)) ret = fsm::fail;
    } else {
        if(!stateMoveJointsPos_->SetJointsGoal(jointsPos_, moveType_)) ret = fsm::fail;
    }

    if (!enableObstacleAvoidance) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![MoveJointsPos::Execute] MoveJointPos command enableObstacleAvoidance = " << enableObstacleAvoidance << std::endl;
    }
    stateMoveJointsPos_->enableObstacleAvoidance = enableObstacleAvoidance;

    if(ret == fsm::ok) {
        ret = fsm_->SetNextState(appleRobot::ID::states::move_joints_pos);
        std::cout << "[MoveJointsPos] Ref: " << futils::STLVectorToString(jointsPos_, ',') << std::endl;
    } else {
        ROS_ERROR("Error in Setting Joint Goal (check command parameters)");
    }

    return ret;
}

void MoveJointsPos::SetState(std::shared_ptr<states::GenericState> state)
{
    stateMoveJointsPos_ = std::dynamic_pointer_cast<states::MoveJointsPos>(state);
}

void MoveJointsPos::SetNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
    nh = nodeHandle;
}


void MoveJointsPos::SetJointsGoal(std::vector<double> &joints_pos, std::string &move_type) {
    jointsPos_ = joints_pos;
    moveType_ = move_type;
    singleJointMove = false;
}

void MoveJointsPos::SetSingleJointGoal(std::vector<double> &joints_pos, int i, std::string &move_type) {
    jointsPos_ = joints_pos;
    moveType_ = move_type;
    jointIndex_ = i;
    singleJointMove = true;
}

}

