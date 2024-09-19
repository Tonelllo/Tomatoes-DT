#include "commands/command_move_cartesian.hpp"

#include "tomato_detection/DriverCommand.h"

namespace commands {

MoveCartesian::MoveCartesian() {}

MoveCartesian::~MoveCartesian() {}

fsm::retval MoveCartesian::Execute()
{
    auto ret = fsm::ok;

    if(!stateMoveCartesian_->SetCartesianGoal(targetXYZ_, targetRPY_, moveType_, frameType_)) ret = fsm::fail;

    if(ret == fsm::ok) {
        fsm_->SetNextState(appleRobot::ID::states::move_cartesian);
        std::cout << "[MoveJointsPos::SetCartesianGoal] "
                  << "MoveType: " << moveType_ << ", FrameType: " << frameType_ << std::endl
                  << "xyz: " << futils::STLVectorToString(targetXYZ_, ',') << " | "
                  << "rpy: " << futils::STLVectorToString(targetRPY_, ',')  << std::endl;
    } else {
        ROS_ERROR("Error in SetCartesianGoal (check command parameters)");
    }

    return ret;
}

void MoveCartesian::SetState(std::shared_ptr<states::GenericState> state)
{
    stateMoveCartesian_ = std::dynamic_pointer_cast<states::MoveCartesian>(state);
}

void MoveCartesian::SetNodeHandle(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
    nh = nodeHandle;
}

void MoveCartesian::SetCartesianGoal(boost::array<double, 3> target_xyz, boost::array<double, 3> target_rpy,
                                     const std::string &move_type, int frame_type)
{
    targetXYZ_ = target_xyz;
    targetRPY_ = target_rpy;
    moveType_  = move_type;
    frameType_ = frame_type;
}

}

