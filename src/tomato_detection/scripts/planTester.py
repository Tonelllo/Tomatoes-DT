import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseArray, Vector3
from moveit_msgs.msg import MoveGroupGoal, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint
import actionlib
import sys

rospy.init_node("plan_tester")

pp = PoseArray()
p = Pose()
p.position.x = 0.9
p.position.y = 0.0
p.position.z = 0.9
p1 = Pose()
p1.position.x = 0.8
p1.position.y = 0.0
p1.position.z = 0.8
p2 = Pose()
p2.position.x = 0.7
p2.position.y = 0.0
p2.position.z = 0.7
pp.poses.append(p)

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
names = robot.get_group_names()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm_torso")

v1 = Vector3()
v1.x = 0.9
v1.y = 0.0
v1.z = 0.9

pc = PositionConstraint()
pc.header.frame_id = "base_footprint"
pc.link_name = "gripper_link"
pc.target_point_offset = v1
pc.weight = 100

pc1 = PositionConstraint()
pc1.header.frame_id = "base_footprint"
pc1.link_name = "gripper_link"
pc1.target_point_offset.x = 0.8
pc1.target_point_offset.y = 0.0
pc1.target_point_offset.z = 0.8
pc1.weight = 100

oc = OrientationConstraint()
oc.header.frame_id = "base_footprint"
oc.link_name = "gripper_link"
oc.orientation.x = 0.0
oc.orientation.y = 0.0
oc.orientation.z = 0.0
oc.orientation.w = 0.1
oc.weight = 100

c1 = Constraints()
c1.position_constraints = [pc, pc1]
c1.orientation_constraints = [oc, oc]


mgg = MoveGroupGoal()

mgg.request.start_state = move_group.get_current_state()
mgg.request.group_name = "arm_torso"
mgg.request.num_planning_attempts = 1
mgg.request.max_velocity_scaling_factor = 1
mgg.request.max_acceleration_scaling_factor = 1
mgg.request.allowed_planning_time = 5.0
mgg.request.planner_id = ""
mgg.request.goal_constraints = [c1]
mgg.planning_options.plan_only = True
mgg.planning_options.look_around = False
mgg.planning_options.planning_scene_diff.is_diff = True
mgg.planning_options.planning_scene_diff.robot_state.is_diff = True

tester = actionlib.SimpleActionClient("/move_group", MoveGroupAction)
tester.wait_for_server()

rospy.loginfo("Send goal")
tester.send_goal(mgg)
tester.wait_for_result()
rospy.loginfo("Sent goal")

rospy.spin()

# 00117     joint_state_target_.reset(new robot_state::RobotState(getRobotModel()));
# 00118     joint_state_target_->setToDefaultValues();
# 00119     active_target_ = JOINT;
# 00120     can_look_ = false;
# 00121     can_replan_ = false;
# 00122     replan_delay_ = 2.0;
# 00123     goal_joint_tolerance_ = 1e-4;
# 00124     goal_position_tolerance_ = 1e-4;     // 0.1 mm
# 00125     goal_orientation_tolerance_ = 1e-3;  // ~0.1 deg
# 00126     planning_time_ = 5.0;
# 00127     num_planning_attempts_ = 1;
# 00128     max_velocity_scaling_factor_ = 1.0;
# 00129     max_acceleration_scaling_factor_ = 1.0;
# 00130     initializing_constraints_ = false;
