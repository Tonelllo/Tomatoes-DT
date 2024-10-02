import moveit_commander
from geometry_msgs.msg import PoseStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, PlanningSceneComponents
from geometry_msgs.msg import Point, Pose
import sys
import rospy
import copy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib_msgs.msg import GoalStatusArray
import actionlib
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from visualization_msgs.msg import Marker
import math
from enum import Enum
import numpy as np
from tomato_detection.srv import BestPos
from threading import Lock
from tomato_detection.srv import LatestTomatoPositions
from queue import Queue
import threading

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("plan_tester")
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm_torso")
move_group.set_planner_id("KPIECEkConfigDefault")
move_group.set_planning_time(3)

status_mutex = Lock()
status_array = []


# def statusCallback(status):
#     global status_array
#     status_mutex.acquire()
#     status_array = status.status_list
#     status_mutex.release()


def getNewJointpos(move_group_joint_pos, latest_planned_state):
    index = 0
    pcount = 0
    amg = np.asarray(move_group_joint_pos)
    alp = np.asarray(latest_planned_state.joint_state.position)
    for name in latest_planned_state.joint_state.name:
        if name in ["torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]:
            alp[index] = amg[pcount]
            pcount += 1
        index += 1
    return tuple(alp)


def worker():
    while True:
        (s, t, tt, e) = future_plans.get(block=True)
        move_group.execute(t, wait=True)
        print("executing while planning")
        if future_plans.empty():
            break


BASKET_JOINT_POSITION = [0.10, 1.47, 0.16, 0.0, 2.22, -1.9, -0.48, -1.39]

goal_tomato = PoseStamped()
goal_tomato.header.frame_id = "base_footprint"
goal_tomato.pose.position.x = 0.8
goal_tomato.pose.position.y = 0.0
goal_tomato.pose.position.z = 0.8
goal_tomato.pose.orientation.x = 0
goal_tomato.pose.orientation.y = 0
goal_tomato.pose.orientation.z = 0
goal_tomato.pose.orientation.w = 1

future_plans = Queue()
latest_planned_state = move_group.get_current_state()
move_group.set_start_state_to_current_state()
first = True

x = threading.Thread(target=worker, args=())
x.start()

for i in range(0, 2):
    if first:
        first = False
    else:
        move_group.set_start_state(latest_planned_state)

    move_group.set_joint_value_target(BASKET_JOINT_POSITION)
    next_plan = move_group.plan()
    (s, t, tt, e) = next_plan
    future_plans.put(next_plan)

    aux = t.joint_trajectory.points[-1]
    at = getNewJointpos(aux.positions, latest_planned_state)
    latest_planned_state.joint_state.position = at

    move_group.set_start_state(latest_planned_state)
    move_group.set_pose_target(goal_tomato)
    # move_group.set_random_target()

    next_plan = move_group.plan()
    (s, t, tt, e) = next_plan
    future_plans.put(next_plan)

    aux = t.joint_trajectory.points[-1]
    at = getNewJointpos(aux.positions, latest_planned_state)
    latest_planned_state.joint_state.position = at

    rospy.sleep(3)


x.join()

# while not future_plans.empty():
#     print("Executing after planning")
#     (s, t, tt, e) = future_plans.get(block=True)
#     move_group.execute(t, wait=True)

# status_sub = rospy.Subscriber(
#     "/move_group/status", GoalStatusArray, statusCallback)

# rospy.spin()


# /home/tonello/TiagoWs/src/tiago_moveit_config/config/ompl_planning.yaml
