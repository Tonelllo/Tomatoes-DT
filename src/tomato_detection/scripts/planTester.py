import moveit_commander
import sys
import rospy
import numpy as np
from threading import Lock
from geometry_msgs.msg import PoseStamped
from queue import Queue
import threading

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("plan_tester")
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm_torso")
move_gripper = moveit_commander.MoveGroupCommander("gripper")
move_gripper.set_planner_id("KPIECEkConfigDefault")
move_group.set_planner_id("KPIECEkConfigDefault")
move_group.set_planning_time(3)
ARM_TORSO = ["torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
GRIPPER = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
HEAD = ["head_1_joint", "head_2_joint"]

"""
   - caster_back_left_1_joint
    - caster_back_left_2_joint
    - caster_back_right_1_joint
    - caster_back_right_2_joint
    - caster_front_left_1_joint
    - caster_front_left_2_joint
    - caster_front_right_1_joint
    - caster_front_right_2_joint
    - suspension_left_joint
    - wheel_left_joint
    - suspension_right_joint
    - wheel_right_joint
    - torso_lift_joint
    - arm_1_joint
    - arm_2_joint
    - arm_3_joint
    - arm_4_joint
    - arm_5_joint
    - arm_6_joint
    - arm_7_joint
    - gripper_left_finger_joint
    - gripper_right_finger_joint
    - head_1_joint
    - head_2_joint
"""

def getNewJointpos(joint_names, move_group_joint_pos, latest_planned_state):
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


def worker():
    while True:
        t = future_plans.get(block=True)
        move_group.execute(t, wait=True)
        print("executing while planning")
        if future_plans.empty():
            break


BASKET_JOINT_POSITION = [0.10, 1.47, 0.16, 0.0, 2.22, -1.9, -0.48, -1.39]

goal_tomato = PoseStamped()
goal_tomato.header.frame_id = "base_footprint"
goal_tomato.pose.position.x = 0.7
goal_tomato.pose.position.y = 0.0
goal_tomato.pose.position.z = 0.7
goal_tomato.pose.orientation.x = 0
goal_tomato.pose.orientation.y = 0
goal_tomato.pose.orientation.z = 0
goal_tomato.pose.orientation.w = 1

future_plans = Queue()
latest_planned_state = move_group.get_current_state()
# print(latest_planned_state)
move_group.set_start_state_to_current_state()
first = True

x = threading.Thread(target=worker, args=())
x.start()

for i in range(0, 5):
    if first:
        first = False
    else:
        move_group.set_start_state(latest_planned_state)

    move_group.set_joint_value_target(BASKET_JOINT_POSITION)
    next_plan = move_group.plan()
    (s, t, tt, e) = next_plan
    future_plans.put(t)

    aux = t.joint_trajectory.points[-1]
    at = getNewJointpos(ARM_TORSO, aux.positions, latest_planned_state)
    latest_planned_state.joint_state.position = at

    move_gripper.set_start_state(latest_planned_state)
    move_gripper.set_joint_value_target([0.002, 0.002])
    next_plan = move_gripper.plan()
    (s, t, tt, e) = next_plan
    future_plans.put(t)
    aux = t.joint_trajectory.points[-1]
    at = getNewJointpos(GRIPPER, aux.positions, latest_planned_state)
    latest_planned_state.joint_state.position = at

    move_group.set_start_state(latest_planned_state)
    (traj, frac) = move_group.compute_cartesian_path([goal_tomato.pose], 0.01, 0)
    future_plans.put(traj)
    print("full plan")

    aux = traj.joint_trajectory.points[-1]
    at = getNewJointpos(ARM_TORSO, aux.positions, latest_planned_state)
    latest_planned_state.joint_state.position = at

    move_gripper.set_start_state(latest_planned_state)
    move_gripper.set_joint_value_target([0.044, 0.044])
    next_plan = move_gripper.plan()
    (s, t, tt, e) = next_plan
    future_plans.put(t)
    aux = t.joint_trajectory.points[-1]
    at = getNewJointpos(GRIPPER, aux.positions, latest_planned_state)
    latest_planned_state.joint_state.position = at

    rospy.sleep(3)


x.join()


# /home/tonello/TiagoWs/src/tiago_moveit_config/config/ompl_planning.yaml
