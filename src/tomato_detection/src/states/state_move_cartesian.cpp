#include "states/state_move_cartesian.hpp"



namespace states {

MoveCartesian::MoveCartesian() { }

MoveCartesian::~MoveCartesian() { }

bool MoveCartesian::ConfigureStateFromFile(libconfig::Config& confObj)
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

fsm::retval MoveCartesian::OnEntry()
{
    //std::cout << tc::mag << "ON ENTRY (MOVE CARTESIAN): " << fsm_->GetNextStateName() << "." << tc::none << std::endl;

    cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(appleRobot::ID::Tasks::ToolCartesianDistance)->second.task);
    cartesianOrientationTask_ = std::dynamic_pointer_cast<ikcl::CartesianOrientation>(tasksMap.find(appleRobot::ID::Tasks::ToolCartesianOrientation)->second.task);

    //ctrlInfo->cartesianGoalFrame = robotModel->TransformationMatrix(ctrlInfo->toolID);

    ok_ = true;

    if (actionManager->SetAction(appleRobot::ID::Actions::MoveTool, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }

    cnt = 0;
}

bool MoveCartesian::SetCartesianGoal(boost::array<double, 3> target_xyz, boost::array<double, 3> target_rpy, const std::string &move_type, int frame_type)
{

    Eigen::Map<Eigen::Vector3d> translGoal(target_xyz.data(), 3);
    rml::EulerRPY rpyGoal(Eigen::Map<Eigen::Vector3d>(target_rpy.data(), 3));
    Eigen::TransformationMatrix goalFrame;
    goalFrame.TranslationVector(translGoal);
    goalFrame.RotationMatrix(rpyGoal.ToRotationMatrix());

    //std::cout << "SetCartGoal 1" << std::endl;
    //TODO Restore the following:
    if (move_type == tomato_detection::ControlCommandRequest::ABSMOVE) {
        //std::cout << "SetCartGoal: ABSMOVE" << std::endl;
        robotInfo->cartesianGoalFrame = goalFrame;

    } else if (move_type == tomato_detection::ControlCommandRequest::RELMOVE) {
        //std::cout << "SetCartGoal: RELMOVE" << std::endl;
        std::string refFrameID;

        switch (frame_type) {
        case tomato_detection::ControlCommandRequest::EE:
            refFrameID = robotInfo->toolID;
            break;
        case tomato_detection::ControlCommandRequest::BASE:
            ///! TODO FIX!!
            refFrameID = rml::FrameID::Joint + "0";
            break;
        case tomato_detection::ControlCommandRequest::LOCAL:
            ROS_ERROR("Unsupported: appleRobot_msgs::ControlCommandRequest::frame_type::LOCAL");
            return fsm::fail;
            //break;
        case tomato_detection::ControlCommandRequest::WORLD:
            refFrameID = rml::FrameID::WorldFrame;
            break;
        }

        Eigen::TransformationMatrix relativeFrame = robotModel->TransformationMatrix(refFrameID);

        robotInfo->cartesianGoalFrame = relativeFrame * goalFrame;


    } else {
        ROS_ERROR_STREAM("[MoveCartPos] Wrong move type (recv: " << move_type << " but it should have been " << tomato_detection::ControlCommandRequest::ABSMOVE << 
            " or " << tomato_detection::ControlCommandRequest::RELMOVE << ")");
        return false;
    }

    moveTimer_.Start();

    return true;
}

fsm::retval MoveCartesian::Execute()
{
    cnt++;
    auto enableDbgPrnt = cnt % 100 == 0;

    //auto cartError = rml::CartesianError(robotModel->TransformationMatrix(robotInfo->toolID,rml::FrameID::WorldFrame), robotInfo->cartesianGoalFrame);
    auto cartError = rml::CartesianError(robotModel->TransformationMatrix(robotInfo->toolID), robotInfo->cartesianGoalFrame);

    if (enableDbgPrnt) {
        std::cerr << tc::yellow << "[MoveCartesian::Execute] TASK INFO Start..." << tc::none << std::endl;
        //std::cerr << tc::magL << "[MoveCartesian::Execute] Current joint pos = " << robotInfo->armModel_.JointsPosition().transpose() << tc::none << std::endl;
        std::cerr << tc::magL << "[MoveCartesian::Execute] wTgoal = " << std::endl << robotInfo->cartesianGoalFrame << std::endl;
        std::cerr << tc::magL << "[MoveCartesian::Execute] wTtool = " << std::endl << robotModel->TransformationMatrix(rml::FrameID::WorldFrame, robotInfo->toolID) << std::endl;
        std::cerr << tc::yellow << "[MoveCartesian::Execute] Cartesian error = " << cartError.transpose();
        std::cerr << " (lin err is " << cartError.LinearVector().cwiseAbs().norm() << 
        ", ang err is " << cartError.AngularVector().cwiseAbs().norm() << ")" << tc::none << std::endl;
        auto clrTime = tc::bluL;
        if (conf->cartesianMoveTimeout - moveTimer_.Elapsed() < 5) clrTime = tc::redL;
        else if (conf->cartesianMoveTimeout - moveTimer_.Elapsed() < 10) clrTime = tc::yellow;
        std::cout << clrTime << "[MoveCartesian::Execute] Elapsed time is " << moveTimer_.Elapsed() << " out of " <<
            conf->cartesianMoveTimeout << tc::none << std::endl;
    }
    cartesianDistanceTask_->SetTargetDistance(cartError.LinearVector(), rml::FrameID::WorldFrame);
    cartesianOrientationTask_->SetTargetFrame(robotInfo->cartesianGoalFrame, rml::FrameID::WorldFrame);

    cartesianDistanceTask_->Update();
    cartesianOrientationTask_->Update();

    if ((cartError.LinearVector().cwiseAbs().norm() < conf->linearErrorThreshold)
            && true) { //(cartError.AngularVector().cwiseAbs().norm() < conf->angularErrorThreshold) ) {
        std::cout << tc::greenL << "[KCL]: Cartesian Position Reached (lin err is " << cartError.LinearVector().cwiseAbs().norm() << ")." << tc::none << std::endl;
        fsm_->SetNextState(appleRobot::ID::states::idle);
        ok_ = true;
    } else if (moveTimer_.Elapsed() > conf->cartesianMoveTimeout) {
        std::cout << tc::redL << "[KCL]: Cartesian Move Timeout Reached." << tc::none << std::endl;
        fsm_->SetNextState(appleRobot::ID::states::idle);
        ok_ = false;
    }

    return fsm::ok;
}

fsm::retval MoveCartesian::OnExit()
{
    moveTimer_.Stop();
    //std::cout << "moveTimer_.Stop()" << std::endl;

    return fsm::ok;
}



}
