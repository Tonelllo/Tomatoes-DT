import moveit_commander
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from tomato_trajectory_splicer.srv import SpliceService
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("plan_tester")
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm_torso")
move_group.set_planner_id("KPIECEkConfigDefault")
move_group.set_planning_time(3)
move_group.set_max_velocity_scaling_factor(1.0)
move_group.set_max_acceleration_scaling_factor(1.0)

start = PoseStamped()
start.header.frame_id = "base_footprint"
start.pose.position.x = 0.6
start.pose.position.y = 0.4
start.pose.position.z = 0.6
start.pose.orientation.x = 0
start.pose.orientation.y = 0
start.pose.orientation.z = 0
start.pose.orientation.w = 1

med = PoseStamped()
med.header.frame_id = "base_footprint"
med.pose.position.x = 0.6
med.pose.position.y = 0.0
med.pose.position.z = 0.6
med.pose.orientation.x = 0
med.pose.orientation.y = 0
med.pose.orientation.z = 0
med.pose.orientation.w = 1

end = PoseStamped()
end.header.frame_id = "base_footprint"
end.pose.position.x = 0.8
end.pose.position.y = 0.0
end.pose.position.z = 0.6
end.pose.orientation.x = 0
end.pose.orientation.y = 0
end.pose.orientation.z = 0
end.pose.orientation.w = 1

ARM_TORSO = ["torso_lift_joint", "arm_1_joint", "arm_2_joint",
             "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]


def getNewJointPos(joint_names, move_group_joint_pos, latest_planned_state):
    index = 0
    pcount = 0
    amg = np.asarray(move_group_joint_pos)
    alp = np.asarray(latest_planned_state.joint_state.position)
    for name in latest_planned_state.joint_state.name:
        if name in joint_names:
            alp[index] = amg[pcount]
            pcount += 1
        index += 1
    return tuple(alp)


move_group.set_pose_target(start)
move_group.go(wait=True)

baseS = move_group.get_current_state()
getSplicedTraj = rospy.ServiceProxy(
    "/tomato_sync/getSplicedTraj", SpliceService)
(traj1, frac) = move_group.compute_cartesian_path(
    [med.pose], 0.01, False)

at = getNewJointPos(
    ARM_TORSO, traj1.joint_trajectory.points[-1].positions, baseS)
baseS.joint_state.position = at
move_group.set_start_state(baseS)

(traj2, frac) = move_group.compute_cartesian_path(
    [end.pose], 0.01, False)

# move_group.execute(traj1, wait=True)
# move_group.execute(traj2, wait=True)

traj3 = getSplicedTraj(traj1, traj2)
move_group.execute(traj3.res, wait=True)
