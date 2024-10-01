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
from tomato_detection.srv import LatestTomatoPositions

GROUP_NAME = "arm_torso"
TARGET_OFFSET = 0.21
APPROACH_OFFSET = 0.30
AVOID_COLLISION_SPHERE_RAIDUS = 0.07
EFFORT = 0.3
CARTESIAN_FAILURE_THRESHOLD = 0.7
OPEN_GRIPPER_POS = 0.05
DISTANCE_THRESHOLD = 0.05
PLANNING_TIMEOUT = 0.5
BASKET_JOINT_POSITION = [0.10, 1.47, 0.16, 0.0, 2.22, -1.9, -0.48, -1.39]

# These settings generate 20 grasp positions
L_ANGLE = 60
R_ANGLE = 60
T_ANGLE = 0
D_ANGLE = 60
RL_STEP = 30
TD_STEP = 20


class States(Enum):
    """States for the state machine."""

    GET_FIRST_TOMATOES = 0
    PLAN_APPROACH = 1
    PLAN_PICK = 2
    PLAN_GRAB = 3
    PLAN_BACK = 4
    PLAN_HOME = 5
    PLAN_NEXT_TOMATO = 6
    PLAN_RELEASE = 7
    HOME = 8
    EXECUTING_MOVEMENT = 9


def closeGripper(tomato_radius):
    """
    Close gripper.

    :param float tomato_radius: Radius of the tomato to grab.
    """

    # move_group.attach_object("tarnet_tomato", "gripper_link")

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "gripper_left_finger_joint", "gripper_right_finger_joint"]

    tomato_radius -= 0.01

    point = JointTrajectoryPoint()
    point.positions = [tomato_radius, tomato_radius]
    point.effort = [EFFORT, EFFORT]
    point.time_from_start = rospy.Duration(1.0)
    goal.trajectory.points.append(point)

    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def openGripper():
    """Open gripper."""
    # move_group.detach_object("target_tomato")
    # scene.remove_world_object("target_tomato")

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "gripper_left_finger_joint", "gripper_right_finger_joint"]

    point = JointTrajectoryPoint()
    point.positions = [OPEN_GRIPPER_POS, OPEN_GRIPPER_POS]
    point.time_from_start = rospy.Duration(1.0)
    goal.trajectory.points.append(point)

    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def addBasket():
    """Add the tomato basket to the scene."""
    box_position = PoseStamped()
    box_position.header.frame_id = "base_footprint"
    box_position.pose.position.x = 0.4
    box_position.pose.position.y = 0
    box_position.pose.position.z = 0.1
    box_position.pose.orientation.w = 1.0
    scene.add_box("tomato_basket", box_position, (0.20, 0.40, 0.20))


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
    rospy.sleep(2)


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
    head_goal.min_duration = rospy.Duration(1)
    head_goal.max_velocity = 0.50
    head_goal.target = tomato_point
    rospy.loginfo("Waiting for head to position")
    point_head_client.send_goal(head_goal)
    # point_head_client.wait_for_result()
    # rospy.loginfo("Head positioned")


def resetHead():
    """Reset the head at the initial position that was calculated beeing the best."""
    best_head_tilt = getBestHeadPos().bestpos
    point_head_client.cancel_all_goals()
    head_goal = FollowJointTrajectoryGoal()
    look_point = JointTrajectoryPoint()
    look_point.positions = [0.0, best_head_tilt]
    look_point.velocities = [0.0, 0.0]
    look_point.time_from_start = rospy.Duration(3)
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
        rospy.logwarn("All reachable tomatoes have been picked")
        rospy.signal_shutdown("FINISHED TOMATOES")
        sys.exit()

    return lt[index]


def planNextApproach(goal_pose, radius):
    poses = generate_grasp_poses(goal_pose, APPROACH_OFFSET)
    gposes = generate_grasp_poses(goal_pose, TARGET_OFFSET)

    status_mutex.acquire()
    current_goal_idx = max(len(status_array) - 1, 0)
    status_mutex.release()

    current_best_plan_index = -1

    plans = []
    for pos, gpos in zip(poses, gposes):
        move_group.set_pose_target(pos)
        pplan = move_group.plan()
        (success, trajectory, time, error) = pplan
        if not success:
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

    if len(plans) == 0:
        return False
    (generating_pose, pick_pose, actual_plan) = plans[0]
    (success, trajectory, time, error) = actual_plan
    disableCollisionsAtTarget(goal_pose, radius)  # This is ok as is
    return (success, trajectory, time, error, generating_pose, pick_pose, current_best_plan_index)


status_mutex = Lock()
status_array = []


def statusCallback(status):
    global status_array
    status_mutex.acquire()
    status_array = status.status_list
    status_mutex.release()


def pickTomato():
    """
    State machine that enables the picking of the tomatoes.

    :param tomato_id Int: Id of the tomato to pick
    :param goal_pose Pose: Position of the tomato wrt the base_footprint frame
    :param radius Double: Radius of the tomato to pick
    """
    global status_array
    failed_pick = False
    state = States.GET_FIRST_TOMATOES
    old_state = state
    l_approach_pose = Pose()
    l_pick_pose = Pose()
    tomato_id = None
    goal_pose = None
    radius = None
    next_tomato = None
    initial_robot_state = None

    rospy.sleep(1.0)

    while True:
        if state == States.GET_FIRST_TOMATOES:
            # TODO move to external function
            resetHead()
            openGripper()
            move_group.set_joint_value_target(BASKET_JOINT_POSITION)
            move_group.set_planning_time(3.0)
            suc = move_group.go(wait=True)
            move_group.set_planning_time(PLANNING_TIMEOUT)
            initial_robot_state = move_group.get_current_state()
            if not suc:
                rospy.signal_shutdown("ERROR NOT RECOVERABLE")
                sys.exit()
                rospy.logerr("Start not reachable")

            (poses, tomato_id, radius) = getTomatoPoses()
            current_tomato = getNewValidTomato(poses)
            goal_pose = current_tomato
            lookAtTomato(goal_pose)
            processed_tomatoes.append(
                np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z]))
            state = States.PLAN_APPROACH

        elif state == States.HOME:
            (poses, tomato_id, radius) = getTomatoPoses()
            if next_tomato is False:
                state = States.GET_FIRST_TOMATOES
                next_tomato = None
            else:
                state = States.PLAN_APPROACH

        elif state == States.PLAN_APPROACH:
            old_state = state
            rospy.loginfo("Planning approach for tomato [%d]", tomato_id)
            setMarker(goal_pose.pose)
            # TODO Check for plan fail
            if next_tomato is None:
                rospy.loginfo("First iteration")
                next_tomato = planNextApproach(goal_pose, radius)
                if next_tomato is False:
                    next_tomato = None
                    state = States.GET_FIRST_TOMATOES
                    continue
            lookAtTomato(goal_pose)
            (success, trajectory, time, error, gpos, ppos, idx) = next_tomato
            move_group.execute(trajectory, wait=True)
            # TODO probably not needed
            l_approach_pose = copy.deepcopy(gpos)
            l_pick_pose = copy.deepcopy(ppos)
            rospy.logwarn("Planning approach SUCCESS")

            state = States.EXECUTING_MOVEMENT
        elif state == States.PLAN_PICK:
            # rospy.sleep(1.0)
            old_state = state
            rospy.loginfo("Planning pick for tomato [%d]", tomato_id)
            target = [l_pick_pose]
            move_group.set_start_state(move_group.get_current_state())
            (path, frac) = move_group.compute_cartesian_path(target, 0.01, 0)
            move_group.set_pose_target(l_pick_pose)
            success = move_group.execute(path, wait=True)

            if success and frac > CARTESIAN_FAILURE_THRESHOLD:
                rospy.loginfo("Planning pick SUCCESS")
                state = States.EXECUTING_MOVEMENT
            else:
                # processed_tomatoes.append(
                #     np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z]))
                rospy.logwarn("Planning pick FAIL, frac: ", frac)
                state = States.PLAN_BACK

        elif state == States.PLAN_GRAB:
            old_state = state
            rospy.loginfo("Planning grab for tomato [%d]", tomato_id)
            closeGripper(radius)
            # rospy.sleep(1)
            # TODO how to check if grab was successful
            state = States.PLAN_BACK

        elif state == States.PLAN_BACK:
            old_state = state
            rospy.loginfo("Plannig back")
            target = [l_approach_pose]
            move_group.set_start_state(move_group.get_current_state())
            (path, frac) = move_group.compute_cartesian_path(
                target, 0.01, 0, avoid_collisions=False)
            move_group.set_pose_target(l_approach_pose)
            success = move_group.execute(path, wait=True)

            # Here in any case you go back home
            if success and frac >= CARTESIAN_FAILURE_THRESHOLD:
                rospy.loginfo("Planning back SUCCESS")
                state = States.EXECUTING_MOVEMENT
            else:
                rospy.logwarn("Planning back FAIL")
                state = States.PLAN_HOME

        elif state == States.PLAN_HOME:
            resetHead()
            removeSphere()
            old_state = state
            rospy.loginfo("Planning approach for HOME")
            move_group.set_joint_value_target(BASKET_JOINT_POSITION)
            move_group.set_planning_time(3.0)
            move_group.set_start_state(move_group.get_current_state())
            (s, t, tt, e) = move_group.plan()
            move_group.execute(t, wait=False)
            success = True
            # TODO What if it fails?
            if not success:
                rospy.logfatal("Home not reachable trying random position")
                move_group.set_random_target()
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
                state = States.PLAN_HOME
            else:
                move_group.set_planning_time(PLANNING_TIMEOUT)
                state = States.PLAN_NEXT_TOMATO

        elif state == States.PLAN_NEXT_TOMATO:
            new_tomato = getNewValidTomato(poses)
            tomato_id = new_tomato.pose.orientation.y
            goal_pose = new_tomato
            processed_tomatoes.append(
                np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z]))
            radius = new_tomato.pose.orientation.z
            move_group.set_start_state(initial_robot_state)
            next_tomato = planNextApproach(goal_pose, radius)

            state = States.PLAN_RELEASE

        elif state == States.PLAN_RELEASE:
            res = True
            status_mutex.acquire()
            for status in status_array:
                if status.status not in [3, 4]:
                    res = False
                    break
            status_mutex.release()

            if not res:
                print("Waiting for completion")
                # for a in status_array:
                #     print(a.status)
                state = States.PLAN_RELEASE
                rospy.sleep(1.0)
                continue

            old_state = state
            rospy.loginfo("Planning release for tomato")
            openGripper()
            state = States.HOME

        elif state == States.EXECUTING_MOVEMENT:
            rospy.loginfo("Executing movement")
            move_group.stop()
            move_group.clear_pose_targets()
            state = States(old_state.value + 1)


processed_tomatoes = []


def isRipe(tomato):
    """
    Return True if the tomato is ripe.

    :param tomato Pose: Position of the tomato to reach
    :return: True if ripe
    :rtype: bool
    """
    if int(tomato.orientation.x) == 0:
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


def getTomatoPoses():
    """
    Ros callback for "/tomato_vision_manager/tomato_position" topic.

    :param positions PoseArray: Positions of all the tomatoes with additional
    information in orientation
    """

    positions = getTomatoPosesProxy()
    toReachTS = copy.deepcopy(positions.tomatoes.poses)

    # toReachTS = filter(isRipe, toReachTS)
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
        if not math.isnan(float(pose.position.x)):
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
    return (poses, id, radius / 2)


# radiants
# goal_pose.pose.orientation = tf.transformations.from_quaternion_euler(0, 0, 0)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("positionReacher")


robot = moveit_commander.RobotCommander()
names = robot.get_group_names()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)
# move_group.set_planner_id("LBTRRT")
move_group.set_planner_id("SPARS2")
move_group.set_planning_time(PLANNING_TIMEOUT)
gripper_client = actionlib.SimpleActionClient(
    "/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
gripper_client.wait_for_server()
point_head_client = actionlib.SimpleActionClient(
    "/head_controller/point_head_action", PointHeadAction)
point_head_client.wait_for_server()
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
gripper_pub = rospy.Publisher(
    "/parallel_gripper_controller/command", JointTrajectory, queue_size=2)

getBestHeadPos = rospy.ServiceProxy("/tomato_counting/get_best_tilt", BestPos)
getBestHeadPos.wait_for_service()

head_client = actionlib.SimpleActionClient(
    "/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
head_client.wait_for_server()

rospy.wait_for_service("/tomato_counting/get_best_tilt")
rospy.loginfo("Service get_best_tilt ready")

status_sub = rospy.Subscriber(
    "/move_group/status", GoalStatusArray, statusCallback)

print("Current planner", move_group.get_planner_id())

addBasket()

assert move_group.get_planning_frame() == "base_footprint", "Wrong planning frame"

rospy.wait_for_service("/tomato_vision_manager/tomato_position_service")
getTomatoPosesProxy = rospy.ServiceProxy(
    "/tomato_vision_manager/tomato_position_service", LatestTomatoPositions)

pickTomato()

rospy.spin()

# TODO s
# - In case that no tomato has been found in next_tomato wait for the head
#   to reset before getting a new tomato otherwise nan

# https://ompl.kavrakilab.org/planners.html
