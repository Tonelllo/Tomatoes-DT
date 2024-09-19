#ifndef APPLE_TOPICNAMES_HPP
#define APPLE_TOPICNAMES_HPP

#include <string>

#define ARM_JOINTS 6
#define GRIPPER_CLOSED_SETPOINT 0.0
#define GRIPPER_OPEN_SETPOINT 100.0

namespace appleRobot {

namespace ID {

namespace GTArmState // probably we won't use
{
const std::string OFF = "OFF";
const std::string INIT = "INIT";
const std::string READY = "READY";
const std::string MOVING = "MOVING";
const std::string PARKED = "PARKED";
const std::string PARKING = "PARKING";
const std::string EXITING = "EXITING"; // unparking
const std::string FAILURE = "FAILURE";
const std::string DISABLED = "DISABLED";
//const std::string CALIBRATING = "CALIBRATING";
}

namespace GTGripperState
{
const std::string OFF = "OFF";
const std::string INIT = "INIT";
const std::string READY = "READY";
const std::string MOVING = "MOVING";
const std::string OPEN = "OPEN";
const std::string CLOSED = "CLOSED";
const std::string GRASPED = "GRASPED"; // unparking
const std::string FAILURE = "FAILURE";
const std::string DISABLED = "DISABLED";
}

namespace GTManipulatorFeedback
{
// Start char #
// End char \r\n
const std::string ARMPOS = "ARMPOS";         //
const std::string ARMCURR = "ARMCURR";       //
const std::string ARMSTATUS = "ARMSTATUS";   //-> #ARMSTATUS,ID,GTArmStatus\n
const std::string GRIPPOS = "GRIPPOS";       // 0 - 100
const std::string GRIPCURR = "GRIPCURR";
const std::string GRIPSTATUS = "GRIPSTATUS"; //-> #GRIPSTATUS,ID,GTGripperStatus\n
};

///////////////////////////////////////////////////////////////////////////////////

namespace GTArmCommand
{
const std::string EXIT    = "EXIT";
const std::string PARK    = "PARK";
const std::string STOP    = "STOP";
const std::string ENABLE  = "ENABLE ";
const std::string DISABLE = "DISABLE";
//const std::string INIT
}


namespace GTGripperCommand
{
const std::string STOP     = "STOP";
const std::string SETCURR  = "SETCURR";
}

namespace GTManipulatorCommand
{
// Start char #
// End char \r\n
const std::string ARMDESVEL = "ARMDESVEL";  // #ARMDESVEL,ID,qdot1,qdot2...    |  Unità: deg/sec
const std::string ARMDESPOS = "ARMDESPOS";
const std::string ARMCMD = "ARMCMD";
const std::string GRIPDESPOS = "GRIPDESPOS"; // 0 closed, 100 open
const std::string GRIPCMD = "GRIPCMD";  // #GRIPCMD,SETCURR,150     #GRIPCMD,STOP
};

// IL GRASP SARÀ DUE COMANDI:
// UN SETCURR E UN
// GRIPDESPOS=0

namespace RobotModel {
const std::string AppleRobotBody = "AppleRobotBody";
const std::string AppleRobotArmLeft = "Left";
const std::string AppleRobotArmRight = "Right";
const std::string ToolFrame = "Hand";
}

namespace Actions {
const std::string Idle = "Idle";
const std::string ParkUnpark = "ParkUnpark";
const std::string MoveJointsPosition = "MoveJointsPosition";
const std::string MoveJointsVelocity = "MoveJointsVelocity";
const std::string MoveTool = "MoveTool";
}

namespace Tasks {
const std::string JointsLimit = "JointsLimit";
const std::string ObstacleAvoidance = "ObstacleAvoidance";
const std::string JointsPosition = "JointsPosition";
const std::string JointsVelocity = "JointsVelocity";
const std::string ToolCartesianDistance = "ToolCartesianDistance";
const std::string ToolCartesianOrientation = "ToolCartesianOrientation";
}


namespace commands {
const std::string enable = "enable";
const std::string park = "park";
const std::string unpark = "unpark";
const std::string disable = "disable";
const std::string stop = "stop";
const std::string move_joints_pos = "move_joints_pos";
const std::string move_joints_vel = "move_joints_vel";
const std::string move_joint_i_pos = "move_joint_i_pos";
const std::string move_joint_i_vel = "move_joint_i_vel";
const std::string move_cartesian = "move_cartesian";
const std::string gripper_move = "gripper_move";
const std::string gripper_grasp = "gripper_grasp";
}

namespace states {
const std::string disabled = "disabled";
const std::string parking = "parking";
const std::string parked = "parked";
const std::string unparking = "unparking";
const std::string homing = "homing";
const std::string idle = "idle";
const std::string move_joints_pos = "move_joints_pos";
const std::string move_joints_vel = "move_joints_vel";
const std::string move_cartesian = "move_cartesian";

}

}

namespace topicnames {

//const std::string llc_robot_enable = "/apple_robot/llc/robot_enable";
//const std::string llc_robot_unpark = "/apple_robot/llc/robot_unpark";
//const std::string llc_robot_park = "/apple_robot/llc/robot_park";
const std::string driver_cmd = "robot/driver/commands";
const std::string desired_trajectory = "robot/desired_trajectory";

const std::string state_data = "/joint_states";
const std::string ctrl_data = "robot/ctrl/ctrl_data";
const std::string tpik_action = "robot/ctrl/tpik_action";
const std::string joint_markers = "robot/rviz2/joint_markers";

// Services
const std::string srv_control_command = "robot/srv/control_command";
const std::string srv_rosbag_command = "robot/srv/rosbag_command";

// Tasks
const std::string tasks = "robot/tasks/";

// Best apple
const std::string best_apple = "robot/best_apple/";
const std::string taken_apple_ids = "robot/taken_apple_ids/";
const std::string failed_apple_ids = "robot/failed_apple_ids/";

}
}

#endif // APPLE_TOPICNAMES_HPP
