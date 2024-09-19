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

    //auto cartError = rml::CartesianError(robotModel->TransformationMatrix(robotInfo->toolID,rml::FrameID::WorldFrame), robotInfo->cartesianGoalFrame);
    auto cartError = rml::CartesianError(robotModel->TransformationMatrix(robotInfo->toolID), robotInfo->cartesianGoalFrame);
    std::cerr << "wTgoal = " << std::endl << robotInfo->cartesianGoalFrame << std::endl;

    //cartesianDistanceTask_->SetTargetDistance(cartError.LinearVector(), rml::FrameID::WorldFrame);
    //cartesianOrientationTask_->SetTargetFrame(ctrlInfo->cartesianGoalFrame, rml::FrameID::WorldFrame);

    cartesianDistanceTask_->SetTargetDistance(cartError.LinearVector(), rml::FrameID::WorldFrame);
   //cartesianDistanceTask_->SetTargetDistance()
    cartesianOrientationTask_->SetTargetFrame(robotInfo->cartesianGoalFrame, rml::FrameID::WorldFrame);

    cartesianDistanceTask_->Update();
    cartesianOrientationTask_->Update();

    std::cerr << "CartesianError: " << cartError.transpose() << std::endl;
    std::cerr << "cartesianDistanceTask_->ControlVariable(): " << cartesianDistanceTask_->ControlVariable().transpose() << std::endl;
    std::cerr << "robotModel->TransformationMatrix(ctrlInfo->toolID): " << std::endl << robotModel->TransformationMatrix(robotInfo->toolID) <<  std::endl;
    //std::cout << "Joints Pos: " << robotModel->Arm("youbot")->JointsPosition().transpose() << std::endl;
    std::cerr << "cartesianDistanceTask_->ReferenceRate(): " << cartesianDistanceTask_->ReferenceRate().transpose() << std::endl;
    //std::cout << "cartesianOrientationTask_->Reference(): " << cartesianOrientationTask_->Reference().transpose() << std::endl;
    //std::cout << "***" << std::endl;

    

    if ((cartError.LinearVector().cwiseAbs().norm() < conf->linearErrorThreshold)
            && (cartError.AngularVector().cwiseAbs().norm() < conf->angularErrorThreshold) ) {
        std::cout << "[KCL]: Cartesian Position Reached." << std::endl;
        fsm_->SetNextState(appleRobot::ID::states::idle);
        ok_ = true;
    } else if (moveTimer_.Elapsed() > conf->cartesianMoveTimeout) {
        std::cout << "[KCL]: Cartesian Move Timeout Reached." << std::endl;
        fsm_->SetNextState(appleRobot::ID::states::idle);
        ok_ = false;
    }
   //  std::cerr << "[MoveCartesian::Execute()] Ang Err is " << 
   //      cartError.AngularVector().cwiseAbs().norm() << " with thr " << conf->angularErrorThreshold << std::endl;
   // std::cerr << "[MoveCartesian::Execute()] Lin Err is " << 
       // cartError.LinearVector().cwiseAbs().norm() << " with thr " << conf->linearErrorThreshold << std::endl;


    //std::cout << "moveTimer_.Elapsed(): " << moveTimer_.Elapsed() << std::endl;

    return fsm::ok;
}

fsm::retval MoveCartesian::OnExit()
{
    moveTimer_.Stop();
    //std::cout << "moveTimer_.Stop()" << std::endl;

    return fsm::ok;
}



}
