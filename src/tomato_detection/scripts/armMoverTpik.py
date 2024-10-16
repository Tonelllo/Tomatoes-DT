#!/usr/bin/env python

import moveit_commander
import time
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
from tomato_detection.msg import ControlData

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

taskData: ControlData
taskData = None

class MotionState(Enum):
    MOTION_FINISHED = 1
    MOTION_ONGOING = 2
    MOTION_FAILED = 3
    MOTION_REQUESTED = 4
    UNKNOWN = 5

def TaskDataCallback(data):
    global taskData
    taskData = data

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# MvJnt function to send joint commands
def RequestCartesianMotion(pose, motion_id):
    from tomato_detection.srv import ControlCommand
    rospy.wait_for_service('/left_robot/srv/control_command')
    try:
        controlCommandSrv = rospy.ServiceProxy('/left_robot/srv/control_command', ControlCommand)

        roll, pitch, yaw = euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w] )

        resp = controlCommandSrv(
            command_type = "move_cartesian",
            move_type="absolute",
            joint_setpoint=[],
            joint_index=0,  # Update as needed
            target_position=[pose.position.x, pose.position.y, pose.position.z],  # Update as needed
            target_orientation=[roll, pitch, yaw],  # Update as needed
            frame_type=0,
            id=motion_id,  # Use the motion_id parameter
            gripper_setpoint=0.0,
            grasp_current=0.0,
            enableObstacleAvoidance=False
        )
        print("Joint movement command sent:", resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

# MvJnt function to send joint commands
def RequestJointMotion(joint_coordinates, motion_id):
    from tomato_detection.srv import ControlCommand
    rospy.wait_for_service('/left_robot/srv/control_command')
    try:
        controlCommandSrv = rospy.ServiceProxy('/left_robot/srv/control_command', ControlCommand)
        resp = controlCommandSrv(
            command_type="move_joints_pos",
            move_type="absolute",
            joint_setpoint=joint_coordinates,
            joint_index=0,  # Update as needed
            target_position=[0.0, 0.0, 0.0],  # Update as needed
            target_orientation=[0.0, 0.0, 0.0],  # Update as needed
            frame_type=0,
            id=motion_id,  # Use the motion_id parameter
            gripper_setpoint=0.0,
            grasp_current=0.0,
            enableObstacleAvoidance=False
        )
        print("Joint movement command sent:", resp)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def MoveCartesian(target_pose, motion_id, currId, current_state, time_elapsed, timeout):
    timeElapsedTPIK = -1
    motionStatus = MotionState.MOTION_FAILED
    if time_elapsed > timeout:
        SendStop()
    elif time_elapsed < 0:
        RequestCartesianMotion(target_pose, motion_id)
        timeElapsedTPIK = 0
        print("request done")
        motionStatus = MotionState.MOTION_REQUESTED
    elif "idle" in current_state and int(currId) == int(motion_id):
        timeElapsedTPIK = -1
        motionStatus = MotionState.MOTION_FINISHED
    else:
        timeElapsedTPIK = time_elapsed + 1
        motionStatus = MotionState.MOTION_ONGOING
    return motionStatus, timeElapsedTPIK

def MoveJoints(target_joint_coordinates, motion_id, currId, current_state, time_elapsed, timeout):
    timeElapsedTPIK = -1
    motionStatus = MotionState.MOTION_FAILED
    if time_elapsed > timeout:
        SendStop()
    elif time_elapsed < 0:
        RequestJointMotion(target_joint_coordinates, motion_id)
        timeElapsedTPIK = 0
        print("request done")
        motionStatus = MotionState.MOTION_REQUESTED
    elif "idle" in current_state and int(currId) == int(motion_id):
        timeElapsedTPIK = -1
        motionStatus = MotionState.MOTION_FINISHED
    else:
        timeElapsedTPIK = time_elapsed + 1
        motionStatus = MotionState.MOTION_ONGOING
    return motionStatus, timeElapsedTPIK

class ControllerType(Enum): #LTA
    MOVEIT = 1 #LTA
    TPIK = 2 #LTA
    TPIK_TEST = 3 #LTA

controllerType: ControllerType

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
    point.time_from_start = rospy.Duration(0.5)
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
    point.time_from_start = rospy.Duration(0.5)
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
    global status_array, taskData
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

    timeElapsedTPIK = -1

    rospy.sleep(1.0)

    while True:
        motionStatus = MotionState.UNKNOWN
        taskDataCopy = copy.copy(taskData)
        taskDataCopy: ControlData
        if state == States.GET_FIRST_TOMATOES:
            resetHead()
            openGripper()
            if timeElapsedTPIK < 0:
                print(bcolors.WARNING + "Get first tomatoes" + bcolors.ENDC)
            if taskDataCopy is not None:
                motionStatus, timeElapsedTPIK = MoveJoints(BASKET_JOINT_POSITION, 2, taskDataCopy.idMotion, taskDataCopy.ctrl_state, timeElapsedTPIK, 1000000)

            if (motionStatus == MotionState.MOTION_FINISHED):
                (poses, tomato_id, radius) = getTomatoPoses()
                current_tomato = getNewValidTomato(poses)
                goal_pose = current_tomato
                lookAtTomato(goal_pose)
                processed_tomatoes.append(
                    np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z]))
                print(bcolors.OKGREEN + "Get first tomatoes OK" + bcolors.ENDC)
                state = States.PLAN_APPROACH

        elif state == States.HOME:
            (poses, tomato_id, radius) = getTomatoPoses()
            if next_tomato is False:
                state = States.GET_FIRST_TOMATOES
                next_tomato = None
            else:
                timeElapsedTPIK = -1
                state = States.PLAN_APPROACH
    
        elif state == States.PLAN_APPROACH:
            old_state = state
            if timeElapsedTPIK < 0:
                rospy.loginfo("Planning approach for tomato [%d]", tomato_id)
                print(bcolors.WARNING + "Plan approach" + bcolors.ENDC)
                print(bcolors.OKBLUE + "goal pose is " + str(goal_pose) + bcolors.ENDC)
            setMarker(goal_pose.pose) # rviz
            poses = generate_grasp_poses(goal_pose, APPROACH_OFFSET)
            pose1 = poses[int(len(poses)/2.0)]
            motionStatus, timeElapsedTPIK = MoveCartesian(pose1, 3, taskDataCopy.idMotion, taskDataCopy.ctrl_state, timeElapsedTPIK, 1000000)
            if (motionStatus == MotionState.MOTION_FINISHED):
                print(bcolors.OKGREEN + "Plan approach OK" + bcolors.ENDC)
                state = States.PLAN_PICK
                timeElapsedTPIK = -1

        elif state == States.PLAN_PICK:
            # rospy.sleep(1.0)
            old_state = state
            if timeElapsedTPIK < 0:
                rospy.loginfo("Planning pick for tomato [%d]", tomato_id)
                print(bcolors.WARNING + "Plan pick" + bcolors.ENDC)
            gposes = generate_grasp_poses(goal_pose, TARGET_OFFSET)
            gpose1 = gposes[int(len(poses)/2.0)]
            motionStatus, timeElapsedTPIK = MoveCartesian(gpose1, 4, taskDataCopy.idMotion, taskDataCopy.ctrl_state, timeElapsedTPIK, 1000000)
            if (motionStatus == MotionState.MOTION_FINISHED):
                print(bcolors.OKGREEN + "Plan pick OK" + bcolors.ENDC)
                state = States.PLAN_GRAB
                timeElapsedTPIK = -1

        elif state == States.PLAN_GRAB:
            old_state = state
            rospy.loginfo("Planning grab for tomato [%d]", tomato_id)
            print(bcolors.WARNING + "Plan grab" + bcolors.ENDC)
            time.sleep(1)
            closeGripper(radius)
            time.sleep(1)
            print(bcolors.OKGREEN + "Plan grab OK" + bcolors.ENDC)
            # rospy.sleep(1)
            # TODO how to check if grab was successful
            state = States.PLAN_BACK

        elif state == States.PLAN_BACK:
            if timeElapsedTPIK < 0:
                rospy.loginfo("Plan back")
                print(bcolors.WARNING + "Plan back" + bcolors.ENDC)
            poses = generate_grasp_poses(goal_pose, APPROACH_OFFSET)
            pose1 = poses[int(len(poses)/2.0)]
            motionStatus, timeElapsedTPIK = MoveCartesian(pose1, 5, taskDataCopy.idMotion, taskDataCopy.ctrl_state, timeElapsedTPIK, 1000000)
            if (motionStatus == MotionState.MOTION_FINISHED):
                print(bcolors.OKGREEN + "Plan back OK" + bcolors.ENDC)
                state = States.PLAN_HOME
                timeElapsedTPIK = -1

        elif state == States.PLAN_HOME:
            resetHead()
            #removeSphere()
            old_state = state
            if timeElapsedTPIK < 0:
                rospy.loginfo("Planning approach for HOME")
                print(bcolors.WARNING + "Plan approach for HOME" + bcolors.ENDC)
            motionStatus, timeElapsedTPIK = MoveJoints(BASKET_JOINT_POSITION, 6, taskDataCopy.idMotion, taskDataCopy.ctrl_state, timeElapsedTPIK, 1000000)
            if motionStatus == MotionState.MOTION_FAILED:
                print(bcolors.OKGREEN + "Plan approach for HOME OK" + bcolors.ENDC)
                state = States.PLAN_HOME
                timeElapsedTPIK = -1
            else:
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
            print(bcolors.WARNING + "Plan release" + bcolors.ENDC)
            time.sleep(3)
            openGripper()
            time.sleep(3)
            print(bcolors.OKGREEN + "Plan release OK" + bcolors.ENDC)
            state = States.HOME
        rospy.sleep(0.05)

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
    id = None
    radius = 0.0
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

# tpik state from /left_robot/ctrl/ctrl_data
def TestTPIKServiceCart():
    rospy.wait_for_service('/left_robot/srv/control_command')
    from tomato_detection.srv import ControlCommand
    try:
        controlCommandSrv = rospy.ServiceProxy('/left_robot/srv/control_command', ControlCommand)
        resp1 = controlCommandSrv(command_type = "move_cartesian",
            move_type = "absolute",
            joint_setpoint = [],
            joint_index = 0,
            #target_position = [0.5, -0.3, 0.74],
            target_position = [0.14, -0.72, 0.8],
            target_orientation = [1.548, -0.001, 0.010],
            frame_type = 3,
            id = 0,
            gripper_setpoint = 0.0,
            grasp_current = 0.0,
            enableObstacleAvoidance = False)
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# tpik state from /left_robot/ctrl/ctrl_data
def TestEachJoint(i: int):
    rospy.wait_for_service('/left_robot/srv/control_command')
    jointTarget = [0] * 8
    if i >= 0: jointTarget[i] = 1.5
    from tomato_detection.srv import ControlCommand
    try:
        controlCommandSrv = rospy.ServiceProxy('/left_robot/srv/control_command', ControlCommand)
        resp1 = controlCommandSrv(command_type = "move_joints_pos",
            move_type = "absolute",
            joint_setpoint = jointTarget,
            joint_index = 0,
            target_position = [0.0, 0.0, 0.0],
            target_orientation = [0.0, 0.0, 0.0],
            frame_type = 0,
            id = 0,
            gripper_setpoint = 0.0,
            grasp_current = 0.0,
            enableObstacleAvoidance = False)
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# tpik state from /left_robot/ctrl/ctrl_data
def SendStop():
    return
    rospy.wait_for_service('/left_robot/srv/control_command')
    from tomato_detection.srv import ControlCommand
    try:
        controlCommandSrv = rospy.ServiceProxy('/left_robot/srv/control_command', ControlCommand)
        resp1 = controlCommandSrv(command_type = "idle",
            move_type = "absolute",
            joint_setpoint = [],
            #joint_setpoint = [0.35,0,-0.85,-0.57,0.39,-0.98,0.86,1.24],
            joint_index = 0,
            target_position = [],
            target_orientation = [],
            frame_type = 0,
            id = 1,
            gripper_setpoint = 0.0,
            grasp_current = 0.0,
            enableObstacleAvoidance = False)
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# tpik state from /left_robot/ctrl/ctrl_data
def TestTPIKServiceJoint():
    rospy.wait_for_service('/left_robot/srv/control_command')
    from tomato_detection.srv import ControlCommand
    try:
        controlCommandSrv = rospy.ServiceProxy('/left_robot/srv/control_command', ControlCommand)
        resp1 = controlCommandSrv(command_type = "move_joints_pos",
            move_type = "absolute",
            joint_setpoint = [0,0,0,0,0,0,0,0],
            #joint_setpoint = [0.35,0,-0.85,-0.57,0.39,-0.98,0.86,1.24],
            joint_index = 0,
            target_position = [0.0, 0.0, 0.0],
            target_orientation = [0.0, 0.0, 0.0],
            frame_type = 0,
            id = 1,
            gripper_setpoint = 0.0,
            grasp_current = 0.0,
            enableObstacleAvoidance = False)
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


controllerType = ControllerType.TPIK

if controllerType == ControllerType.TPIK_TEST:
    #TestTPIKServiceCart()
    TestTPIKServiceJoint()
    #TestEachJoint(1)
    #SendStop()
elif controllerType == ControllerType.TPIK:

    rospy.init_node("positionReacher")
    rospy.Subscriber('/left_robot/ctrl/ctrl_data', ControlData, TaskDataCallback, queue_size=1)
    gripper_client = actionlib.SimpleActionClient(
        "/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    gripper_client.wait_for_server()
    print("[TPIK] gripper ok")

    point_head_client = actionlib.SimpleActionClient(
        "/head_controller/point_head_action", PointHeadAction)
    point_head_client.wait_for_server()
    print("[TPIK] point ok")

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
    gripper_pub = rospy.Publisher(
        "/parallel_gripper_controller/command", JointTrajectory, queue_size=2)
    print("[TPIK] marker, gripper pub ok")

    getBestHeadPos = rospy.ServiceProxy("/tomato_counting/get_best_tilt", BestPos)
    getBestHeadPos.wait_for_service()
    print("[TPIK] get best head ok")

    head_client = actionlib.SimpleActionClient(
        "/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    head_client.wait_for_server()
    print("[TPIK] head client ok")

    rospy.wait_for_service("/tomato_counting/get_best_tilt")
    rospy.loginfo("Service get_best_tilt ready")
    print("[TPIK] best tilt ok")

    #addBasket()
    print("[TPIK] add basket ok")

    rospy.wait_for_service("/tomato_vision_manager/tomato_position_service")
    getTomatoPosesProxy = rospy.ServiceProxy(
        "/tomato_vision_manager/tomato_position_service", LatestTomatoPositions)

    pickTomato()
    print("[TPIK] pick tomato ok")

    rospy.spin()


elif controllerType == ControllerType.MOVEIT:
    # radiants
    # goal_pose.pose.orientation = tf.transformations.from_quaternion_euler(0, 0, 0)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("positionReacher")

    robot = moveit_commander.RobotCommander()
    names = robot.get_group_names()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    # move_group.set_planner_id("LBTRRT")
    move_group.set_planner_id("KPIECEkConfigDefault")
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
