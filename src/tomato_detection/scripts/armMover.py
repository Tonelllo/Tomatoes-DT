#!/usr/bin/env python

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
from tomato_detection.srv import LatestTomatoPositions, CurrentVisionState
from tomato_trajectory_splicer.srv import SpliceService
from queue import Queue
from std_srvs.srv import Empty
import threading

GROUP_NAME = "arm_torso"
TARGET_OFFSET = 0.21
APPROACH_OFFSET = 0.33
AVOID_COLLISION_SPHERE_RAIDUS = 0.07
EFFORT = 0.3
CARTESIAN_FAILURE_THRESHOLD = 0.7
OPEN_GRIPPER_POS = 0.044
CLOSE_GRIPPER_POS = 0.001
DISTANCE_THRESHOLD = 0.05
PLANNING_TIMEOUT = 1.0
PLAN_HOME_TIMEOUT = 1.0
FIRST_ITER_PLANNING_TIMEOUT = 1.0
PLAN_HOME_PLANNER = "RRTstarkConfigDefault"
NORMAL_PLANER = "RRTstarkConfigDefault"
BASKET_JOINT_POSITION = [0.133, 0.72, 0.06, 0.50, 1.42, -0.54, 0.56, -0.2]
PRE_APPROACH_POSITION = [0.117, 1.56, -
                         1.087, -3.307,  2.184, -1.184, -0.895, -0.33]
ARM_TORSO = ["torso_lift_joint", "arm_1_joint", "arm_2_joint",
             "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
GRIPPER = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
HEAD = ["head_1_joint", "head_2_joint"]

# These settings generate 20 grasp positions
L_ANGLE = 0
R_ANGLE = 1
T_ANGLE = 45
D_ANGLE = 46
RL_STEP = 30
TD_STEP = 30


class Colors:
    blue = "\033[34m"
    reset = "\033[0m"


planning_mutex = threading.Lock()
status_mutex = Lock()
status_array = []
future_plans = Queue()
future_plans_num = 0
tomato_poses_mutex = Lock()
processed_tomatoes = []


class States(Enum):
    """States for the state machine."""

    RESTART = -2
    WAIT_FOR_SCAN = -1
    RESET = 0
    PLAN_PRE_APPROACH = 1
    PLAN_APPROACH = 2
    PLAN_PICK = 3
    PLAN_GRAB = 4
    PLAN_BACK = 5
    PLAN_PRE_HOME = 6
    PLAN_HOME = 7
    PLAN_NEXT_TOMATO = 8
    PLAN_RELEASE = 9
    HOME = 10
    EXECUTING_MOVEMENT = 11


def closeGripper():
    """
    Close gripper.

    :param float tomato_radius: Radius of the tomato to grab.
    """
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "gripper_left_finger_joint", "gripper_right_finger_joint"]

    # tomato_radius -= 0.01
    tomato_radius = CLOSE_GRIPPER_POS

    point = JointTrajectoryPoint()
    point.positions = [tomato_radius, tomato_radius]
    point.effort = [EFFORT, EFFORT]
    point.time_from_start = rospy.Duration(1, 0)
    goal.trajectory.points.append(point)

    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()
    print("grabbed")
    # rospy.sleep(1)


def openGripper():
    """Open gripper."""

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "gripper_left_finger_joint", "gripper_right_finger_joint"]

    point = JointTrajectoryPoint()
    point.positions = [OPEN_GRIPPER_POS, OPEN_GRIPPER_POS]
    point.time_from_start = rospy.Duration(1, 0)
    goal.trajectory.points.append(point)

    gripper_client.send_goal(goal)
    # rospy.sleep(1)
    print("opened")
    gripper_client.wait_for_result()


def addBasket():
    """Add the tomato basket to the scene."""
    box_position = PoseStamped()
    box_position.header.frame_id = "base_footprint"
    box_position.pose.position.x = 0.32
    box_position.pose.position.y = -0.435
    box_position.pose.position.z = 0.1
    box_position.pose.orientation.w = 1.0
    scene.add_box("tomato_basket", box_position, (0.50, 0.30, 0.20))


def removeBasket():
    scene.remove_world_object("tomato_basket")


def setMarker(pose):
    """
    Display a marken in the rviz visualization.

    :param Pose pose: Position of the marken in the space.
    """
    marker = Marker()
    marker.header.frame_id = "base_footprint"
    marker.header.stamp = rospy.Time.now()
    marker.type = 2
    marker.id = 0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1

    marker.pose = pose

    marker_pub.publish(marker)


def getTrajectoryLen(trajectory):
    """
    Get the estimated time to complete the trajectory.

    :param trajectory JointTrajectory: Planned trajectory
    """
    computed = trajectory.joint_trajectory.points
    if len(computed) > 0:
        return computed[-1].time_from_start
    else:
        return None


def disableCollisionsAtTarget(goal_pose, radius):
    """
    Disables the collisions in an area defined by a sphere.

    This sphere has a radius of AVOID_COLLISION_SPHERE_RAIDUS.
    This in order to allow the approach of the arm to the tomato.

    :param goal_pose Pose: Position of the tomato to reach
    """
    # scene.add_sphere("target_tomato", goal_pose, radius)
    scene.add_sphere("noCollisions", goal_pose, AVOID_COLLISION_SPHERE_RAIDUS)
    diff_scene = PlanningScene()
    diff_scene.is_diff = True
    acm = scene.get_planning_scene(
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
    for elem in acm.entry_values:
        elem.enabled.append(True)
    acm.entry_names.append("noCollisions")
    acm.entry_values.append(AllowedCollisionEntry(
        enabled=[True] * len(acm.entry_names)))
    diff_scene.allowed_collision_matrix = acm
    scene.apply_planning_scene(diff_scene)


def removeSphere():
    """Remove the previously set sphere from the scene."""
    scene.remove_world_object("noCollisions")
    diff_scene = PlanningScene()
    diff_scene.is_diff = True
    acm = scene.get_planning_scene(
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
    for elem in acm.entry_values:
        elem.enabled.pop()
    acm.entry_names.pop()
    acm.entry_values.pop()
    diff_scene.allowed_collision_matrix = acm
    scene.apply_planning_scene(diff_scene)


def normalize(v):
    """Normalize the vector."""
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def quaternion_from_vectors(v0, v1):
    """
    Return the quaternion between a couple of arrays.

    :param v0 Array: Vector1
    :param v1 Array: Vector1
    :return: The resulting quaternion
    :returns: Array
    """
    if type(v0) == Point():
        v0 = [v0.x, v0.y, v0.z]
    if type(v1) == Point():
        v1 = [v1.x, v1.y, v1.z]

    v0 = normalize(v0)
    v1 = normalize(v1)
    c = np.cross(v0, v1)
    d = np.dot(v0, v1)
    try:
        s = math.sqrt((1.0 + d) * 2)
    except ValueError:
        s = 0.0
    if s == 0.0:
        # print "s == 0.0, we cant compute"
        return None  # [0.0, 0.0, 0.0, 1.0]

    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = c[0] / s
    q[1] = c[1] / s
    q[2] = c[2] / s
    q[3] = s / 2.0
    return q


def generate_grasp_poses(object_pose, radius=APPROACH_OFFSET):
    """Generate different grasp positions for the tomato."""
    # http://math.stackexchange.com/questions/264686/how-to-find-the-3d-coordinates-on-a-celestial-spheres-surface
    ori_x = 0.0
    ori_y = 0.0
    ori_z = 0.0
    sphere_poses = []
    rotated_q = quaternion_from_euler(0.0, 0.0, math.radians(180))

    # altitude is yaw
    # NOTE MIN, MAX, STEP
    for altitude in range(180-L_ANGLE, 180+R_ANGLE + 1, RL_STEP):  # NOQA
        altitude = math.radians(altitude)
        # azimuth is pitch
        for azimuth in range(T_ANGLE, D_ANGLE + 1, TD_STEP):  # NOQA
            azimuth = math.radians(azimuth)
            # This gets all the positions
            x = ori_x + radius * math.cos(azimuth) * math.cos(altitude)
            y = ori_y + radius * math.sin(altitude)
            z = ori_z + radius * math.sin(azimuth) * math.cos(altitude)
            # this gets all the vectors pointing outside of the center
            # quaternion as x y z w
            q = quaternion_from_vectors([radius, 0.0, 0.0], [x, y, z])
            # Cannot compute so the vectors are parallel
            if q is None:
                # with this we add the missing arrow
                q = rotated_q
            # We invert the orientations to look inwards by multiplying
            # with a quaternion 180deg rotation on yaw
            q = quaternion_multiply(q, rotated_q)

            # We actually want roll to be always 0.0 so we approach
            # the object with the gripper always horizontal
            # this rotation can be tuned with the dynamic params
            # multiplying later on
            roll, pitch, yaw = euler_from_quaternion(q)
            q = quaternion_from_euler(math.radians(90), pitch, yaw)

            x += object_pose.pose.position.x
            y += object_pose.pose.position.y
            z += object_pose.pose.position.z

            current_pose = Pose(Point(x, y, z), Quaternion(*q))
            current_pose.position.x = x
            current_pose.position.y = y
            current_pose.position.z = z
            sphere_poses.append(current_pose)
    return sphere_poses


def getTrajectoryLen(trajectory):
    """
    Get the estimated time to complete the trajectory.

    :param trajectory JointTrajectory: Planned trajectory
    :return: Time to reach the tomato
    :returns: Double
    """
    computed = trajectory.joint_trajectory.points
    return computed[-1].time_from_start


def lookAtTomato(tomato_position):
    """
    Make the robot look at the given position in the world.
    NOTE that the position has to be with respect to the "base_footprint" frame.

    :param tomato_position PoseStamped: The position of the tomato in the world
    """
    rospy.loginfo("looking at tomato")
    tomato_point = PointStamped()
    tomato_point.point.x = tomato_position.pose.position.x
    tomato_point.point.y = tomato_position.pose.position.y
    tomato_point.point.z = tomato_position.pose.position.z
    tomato_point.header.frame_id = "/base_footprint"

    head_goal = PointHeadGoal()
    head_goal.pointing_frame = "/xtion_rgb_optical_frame"
    head_goal.pointing_axis.x = 0
    head_goal.pointing_axis.y = 0
    head_goal.pointing_axis.z = 1
    head_goal.min_duration = rospy.Duration(1.0)
    head_goal.max_velocity = 0.50
    head_goal.target = tomato_point

    point_head_client.send_goal(head_goal)
    # point_head_client.wait_for_result()
    # rospy.loginfo("Head positioned")


def resetHead():
    """Reset the head at the initial position that was calculated beeing the best."""
    best_head_tilt = getBestHeadPos().bestpos
    best_head_tilt = 0  # TODO fake
    point_head_client.cancel_all_goals()
    head_goal = FollowJointTrajectoryGoal()
    look_point = JointTrajectoryPoint()
    look_point.positions = [0.0, best_head_tilt]
    look_point.velocities = [0.0, 0.0]
    look_point.time_from_start = rospy.Duration(1.0)
    head_goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
    head_goal.trajectory.points = [look_point]
    rospy.loginfo("Resetting head position")
    head_client.send_goal(head_goal)
    # head_client.wait_for_result()
    # rospy.loginfo("Head resetted")


def getNewValidTomato(lt):
    index = 0
    while index < len(lt) and checkDistanceUnderThreshold(lt[index]):
        index += 1

    if index == len(lt):
        rospy.logwarn("All reachable tomatoes have been picked, moving head")
        return False
        # rospy.logwarn("All reachable tomatoes have been picked")
        # rospy.signal_shutdown("FINISHED TOMATOES")
        # sys.exit()

    return lt[index]


def planNextApproach(goal_pose, radius, first_iteration=False, home_state=None):
    pre_poses = generate_grasp_poses(goal_pose, APPROACH_OFFSET)
    pre_gposes = generate_grasp_poses(goal_pose, TARGET_OFFSET)

    if len(pre_poses) != 1:
        f, s = np.array_split(pre_poses, 2)
        f = np.flip(f)
        poses = [val for pair in zip(f, s) for val in pair]
    else:
        poses = pre_poses

    if len(pre_gposes) != 1:
        f, s = np.array_split(pre_gposes, 2)
        f = np.flip(f)
        gposes = [val for pair in zip(f, s) for val in pair]
    else:
        gposes = pre_gposes

    current_goal_idx = max(len(status_array) - 1, 0)

    current_best_plan_index = -1
    succ_count = 0
    count = 0
    plans = []
    for pos, gpos in zip(poses, gposes):
        if rospy.is_shutdown():
            sys.exit()
            break
        move_group.set_start_state_to_current_state()
        move_group.set_pose_target(pos)
        pplan = move_group.plan()
        (success, trajectory, time, error) = pplan
        count += 1
        if success:
            succ_count += 1
        print("Successful plans: ["+str(succ_count) +
              "/12], Processed: ["+str(count)+"/12]")
        if not success:
            rospy.logerr("Planning failed for plan approach")
            continue
        inserted = False
        for idx, plan in enumerate(plans):
            (a, b, aux_plan) = plan
            (iter_success, iter_trajectory,
                iter_time, iter_error) = aux_plan
            if getTrajectoryLen(trajectory) < getTrajectoryLen(iter_trajectory):
                inserted = True
                plans.insert(idx, (pos, gpos, pplan))
                if idx == 0:
                    current_best_plan_index = current_goal_idx
                break
        if not inserted:
            if len(plans) == 0:
                current_best_plan_index = current_goal_idx
            plans.append((pos, gpos, pplan))

        current_goal_idx += 1
        if first_iteration:
            break

    if len(plans) == 0:
        return False
    (generating_pose, pick_pose, actual_plan) = plans[0]
    (success, trajectory, time, error) = actual_plan
    return (success, trajectory, time, error, generating_pose, pick_pose, current_best_plan_index)


def statusCallback(status):
    global status_array
    status_mutex.acquire()
    status_array = status.status_list
    status_mutex.release()


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


def recoverExecutionError():
    rospy.logerr("Recovering from execution error")
    move_group.clear_pose_targets()
    resetHead()
    planning_mutex.acquire()
    move_group.set_start_state_to_current_state()
    move_group.set_joint_value_target(BASKET_JOINT_POSITION)
    move_group.go(wait=True)
    planning_mutex.release()
    openGripper()
    removeSphere()


def pickTomato():
    """
    State machine that enables the picking of the tomatoes.
    """
    global status_array, tomato_poses, future_plans_num
    state = States.RESET
    l_approach_pose = Pose()
    l_pick_pose = Pose()
    goal_pose = None
    next_tomato = None
    first_iter = False
    HOME_STATE = None

    tomato_poses_mutex.acquire()
    tomato_poses = getTomatoPoses()
    tomato_poses_mutex.release()

    while not rospy.is_shutdown():

        if state == States.RESET:
            print(f"{Colors.blue}Planning reset{Colors.reset}")
            resetHead()
            openGripper()
            move_group.set_joint_value_target(BASKET_JOINT_POSITION)
            move_group.set_planning_time(3.0)
            move_group.set_start_state_to_current_state()
            suc = move_group.go(wait=True)
            move_group.set_planning_time(PLANNING_TIMEOUT)

            if not suc:
                rospy.signal_shutdown("ERROR NOT RECOVERABLE")
                sys.exit()
                rospy.logerr("Start not reachable")

            state = States.PLAN_PRE_APPROACH
            res = input("Ready to start? [Y/n]: ")
            if res.lower() == "n":
                rospy.signal_shutdown("FINISHED TOMATOES")
                sys.exit(0)

            tomato_poses_mutex.acquire()
            tomato_poses = getTomatoPoses()
            tomato_poses_mutex.release()

        elif state == States.HOME:
            break
        elif state == States.PLAN_PRE_APPROACH:
            print(f"{Colors.blue}Planning pre approach{Colors.reset}")
            goal_pose = getNewValidTomato(tomato_poses)

            if goal_pose is False:
                state = States.RESTART
                continue

            move_group.set_joint_value_target(PRE_APPROACH_POSITION)
            move_group.set_start_state_to_current_state()
            succ = move_group.go()
            if succ is False:
                rospy.logerr("IMPOSSIBLE TO REACH PREAPPROACh")
                state = States.PLAN_PRE_APPROACH
                continue

            processed_tomatoes.append(
                np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z]))

            state = States.PLAN_APPROACH
        elif state == States.PLAN_APPROACH:
            print(f"{Colors.blue}Planning approach{Colors.reset}")
            setMarker(goal_pose.pose)

            next_tomato = planNextApproach(
                goal_pose, AVOID_COLLISION_SPHERE_RAIDUS, first_iteration=first_iter, home_state=HOME_STATE)
            first_iter = False

            if next_tomato is False:  # TODO check
                rospy.logerr("next tomato is false")
                continue
            (success, traj, time, error, gpos, ppos, idx) = next_tomato

            move_group.set_start_state_to_current_state()
            success = move_group.execute(traj)
            if not success:
                rospy.logerr("error in execution")
                state = States.PLAN_APPROACH

            # LATER
            # TODO probably not needed
            l_approach_pose = copy.deepcopy(gpos)
            l_pick_pose = copy.deepcopy(ppos)

            state = States.PLAN_PICK
        elif state == States.PLAN_PICK:
            print(f"{Colors.blue}Planning pick{Colors.reset}")
            move_group.set_start_state_to_current_state()
            (traj, frac) = move_group.compute_cartesian_path(
                [l_pick_pose], 0.01, avoid_collisions=False)
            if frac >= CARTESIAN_FAILURE_THRESHOLD:
                succ = move_group.execute(traj)
                if not succ:
                    state = States.PLAN_BACK
            else:
                rospy.logwarn("Plan pick Failed")
                state = States.PLAN_BACK
                continue
            state = States.PLAN_GRAB

        elif state == States.PLAN_GRAB:
            print(f"{Colors.blue}Planning grab{Colors.reset}")
            closeGripper()
            state = States.PLAN_BACK

        elif state == States.PLAN_BACK:
            print(f"{Colors.blue}Planning back{Colors.reset}")
            move_group.set_start_state_to_current_state()
            (traj, frac) = move_group.compute_cartesian_path(
                [l_approach_pose], 0.01, avoid_collisions=False)
            if frac >= CARTESIAN_FAILURE_THRESHOLD:
                succ = move_group.execute(traj)
                if not succ:
                    rospy.loginfo("Execution back failed")
                    rospy.sleep(10000)
            else:
                rospy.loginfo("Planning back failed")
                rospy.sleep(10000)
            state = States.PLAN_PRE_HOME

        elif state == States.PLAN_PRE_HOME:
            print(f"{Colors.blue}Planning pre home{Colors.reset}")
            move_group.set_joint_value_target(PRE_APPROACH_POSITION)
            move_group.set_start_state_to_current_state()
            succ = move_group.go()
            if succ is False:
                rospy.logerr("IMPOSSIBLE TO REACH PREHOME")
            state = States.PLAN_HOME

        elif state == States.PLAN_HOME:
            # resetHead()
            print(f"{Colors.blue}Planning home{Colors.reset}")
            move_group.set_start_state_to_current_state()
            move_group.set_joint_value_target(BASKET_JOINT_POSITION)
            move_group.allow_replanning(True)
            move_group.set_planning_time(PLAN_HOME_TIMEOUT)
            move_group.set_planner_id(PLAN_HOME_PLANNER)
            next_tomato = move_group.plan()
            move_group.set_planner_id(NORMAL_PLANER)
            move_group.allow_replanning(False)
            (s, t, tt, e) = next_tomato
            succ = move_group.execute(t)
            if not s or not succ:
                rospy.loginfo("Errore in plan o execute home")
                rospy.sleep(100000)
            move_group.set_planning_time(PLANNING_TIMEOUT)

            print("plan home succ")
            state = States.PLAN_RELEASE

        elif state == States.PLAN_RELEASE:
            print(f"{Colors.blue}Planning release{Colors.reset}")
            openGripper()
            state = States.PLAN_PRE_APPROACH


def isRipe(tomato):
    """
    Return True if the tomato is ripe.

    :param tomato Pose: Position of the tomato to reach
    :return: True if ripe
    :rtype: bool
    """
    if int(tomato.orientation.x) in [0, 1]:
        return True
    else:
        return False


def checkDistanceUnderThreshold(new_tomato):
    """
    Check if the distance for new_tomato is too close to another.

    If that's the case than they are to be considered the same. This function
    was added because it's not possible to rely on the id of the tomatoes.

    :param new_tomato Pose: Position of the new tomato
    :return: True if the tomato is under DISTANCE_THRESHOLD
    :returns: Bool
    """
    for tomato in processed_tomatoes:
        distance = np.linalg.norm(np.array(
            [new_tomato.pose.position.x, new_tomato.pose.position.y, new_tomato.pose.position.z]) - tomato)
        if distance < DISTANCE_THRESHOLD:
            return True
    return False


def stopService(req):
    move_group.stop()
    rospy.signal_shutdown("RECEIVED STOP")
    return []


def getTomatoPoses():
    """
    Ros callback for "/tomato_vision_manager/tomato_position" topic.

    :param positions PoseArray: Positions of all the tomatoes with additional
    information in orientation
    """

    positions = getTomatoPosesProxy()
    toReachTS = copy.deepcopy(positions.tomatoes.poses)

    tReachTS = filter(isRipe, toReachTS)
    toReach = sorted(toReachTS, key=lambda elem: elem.position.x)

    # index = 0
    # while index < len(toReach) and checkDistanceUnderThreshold(toReach[index]):
    #     index += 1

    # if index == len(toReach):
    #     rospy.logwarn("All reachable tomatoes have been picked")
    #     rospy.signal_shutdown("FINISHED TOMATOES")
    #     sys.exit()

    poses = []
    for pose in toReach:
        if not math.isnan(float(pose.position.x)) and float(pose.position.x) > 0.3:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "base_footprint"
            goal_pose.pose.position.x = pose.position.x
            goal_pose.pose.position.y = pose.position.y
            goal_pose.pose.position.z = pose.position.z
            goal_pose.pose.orientation.x = 0.7071068
            goal_pose.pose.orientation.y = 0
            goal_pose.pose.orientation.z = 0
            goal_pose.pose.orientation.w = 0.7071068
            id = pose.orientation.y
            # NOTE 0 because TIAGO will stop when closing too hard
            radius = 0  # pose.orientation.z
            poses.append(goal_pose)
        else:
            rospy.logerr("Received NaN")

    p1 = PoseStamped()
    p1.pose.position.x = 0.7
    p1.pose.position.y = 0.0
    p1.pose.position.z = 0.7
    p1.pose.orientation.x = 0.7
    p1.pose.orientation.y = 0.0
    p1.pose.orientation.z = 0.0
    p1.pose.orientation.w = 0.7

    p2 = PoseStamped()
    p2.pose.position.x = 0.7
    p2.pose.position.y = 0.2
    p2.pose.position.z = 0.7
    p2.pose.orientation.x = 0.7
    p2.pose.orientation.y = 0.0
    p2.pose.orientation.z = 0.0
    p2.pose.orientation.w = 0.7

    p3 = PoseStamped()
    p3.pose.position.x = 0.7
    p3.pose.position.y = -0.2
    p3.pose.position.z = 0.7
    p3.pose.orientation.x = 0.7
    p3.pose.orientation.y = 0.0
    p3.pose.orientation.z = 0.0
    p3.pose.orientation.w = 0.7

    poses = [p1, p2, p3]
    return poses


# radiants
# goal_pose.pose.orientation = tf.transformations.from_quaternion_euler(0, 0, 0)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("positionReacher")

scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)

# move_group.set_planner_id("PRMstarkConfigDefault") # Good trajectories, better than RRTStar in planning time (1.0)
# move_group.set_planner_id("BKPIECEkConfigDefault") # Can lead to atrocious trajectories. Seems to always find a good trajectory. Fast in planning (1.0)
# move_group.set_planner_id("RRTstarkConfigDefault") # Very good trajectories. Not so fast in planning (1.5)
# move_group.set_planner_id("RRTkConfigDefault") # Best at speed. Shitty trajectories (0.5)
# Best at speed. Shitty trajectories (0.5)
move_group.set_planner_id("RRTConnectkConfigDefault")
# This is probably the one that we are going to use because
# ever if the trajectories are not good it produces many of them
# and it's easier to find one that is not soo bad between them
# move_group.set_planner_id("SemiPersistentLazyPRMstar")
# move_group.set_planner_id("TRRTkConfigDefault") # Good speed and trajectories but shitty in finding trajectories (1.2)
move_group.set_planning_time(FIRST_ITER_PLANNING_TIMEOUT)
# move_group.set_max_velocity_scaling_factor(1.0)
# move_group.set_max_acceleration_scaling_factor(1.0)
move_group.set_max_acceleration_scaling_factor(1)
move_group.set_max_velocity_scaling_factor(1)

getSplicedTraj = rospy.ServiceProxy(
    "/tomato_sync/getSplicedTraj", SpliceService)
getSplicedTraj.wait_for_service()
rospy.loginfo("Splicing is now available")

gripper_client = actionlib.SimpleActionClient(
    "/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
gripper_client.wait_for_server()
rospy.loginfo("Gripper client conected")

point_head_client = actionlib.SimpleActionClient(
    "/head_controller/point_head_action", PointHeadAction)
point_head_client.wait_for_server()
rospy.loginfo("HeadPointer is connected")

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
gripper_pub = rospy.Publisher(
    "/parallel_gripper_controller/command", JointTrajectory, queue_size=2)

getBestHeadPos = rospy.ServiceProxy(
    "/tomato_counting/get_best_tilt", BestPos)
getBestHeadPos.wait_for_service()
rospy.loginfo("get_best_tilt is now ready")

head_client = actionlib.SimpleActionClient(
    "/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
head_client.wait_for_server()
rospy.loginfo("follow_joint_trajectory is now ready")

rospy.wait_for_service("/tomato_counting/get_best_tilt")
rospy.loginfo("Service get_best_tilt ready")

status_sub = rospy.Subscriber(
    "/move_group/status", GoalStatusArray, statusCallback)

print("Current planner", move_group.get_planner_id())

removeBasket()
addBasket()

assert move_group.get_planning_frame() == "base_footprint", "Wrong planning frame"

rospy.wait_for_service("/tomato_vision_manager/tomato_position_service")
rospy.loginfo("tomato_position_service is now ready")

getTomatoPosesProxy = rospy.ServiceProxy(
    "/tomato_vision_manager/tomato_position_service", LatestTomatoPositions)

rospy.Service("arm_mover/stop", Empty, stopService)

restartScanProxy = rospy.ServiceProxy(
    "/tomato_vision_manager/start_scan", Empty)
rospy.wait_for_service("/tomato_vision_manager/start_scan")
rospy.loginfo("start_scan is now ready")

getStateProxy = rospy.ServiceProxy(
    "/tomato_vision_manager/get_state", CurrentVisionState)
rospy.wait_for_service("/tomato_vision_manager/get_state")
rospy.loginfo("get_state is now ready")

pickTomato()

# TODO s
# - In case that no tomato has been found in next_tomato wait for the head
#   to reset before getting a new tomato otherwise nan

# https://ompl.kavrakilab.org/planners.html
