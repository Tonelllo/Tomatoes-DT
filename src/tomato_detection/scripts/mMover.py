#!/usr/bin/env python

import moveit_commander
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import PoseArray, Quaternion
from geometry_msgs.msg import Point, Pose
import sys
import rospy
import copy
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import math
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from moveit_msgs.msg import PickupAction, PickupGoal, Grasp, PlaceGoal, PlaceLocation, GripperTranslation, PlaceAction
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, PlanningSceneComponents
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler, quaternion_multiply
from tf.transformations import euler_from_quaternion
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
from tomato_detection.srv import LatestTomatoPositions
from control_msgs.msg import PointHeadAction, PointHeadGoal

GROUP_NAME = "arm_torso"
TARGET_OFFSET = 0.21
APPROACH_OFFSET = 0.30
AVOID_COLLISION_SPHERE_RADIUS = 0.055
EFFORT = 0.3
CARTESIAN_FAILURE_THRESHOLD = 0.9
OPEN_GRIPPER_POS = 0.05
BASKET_JOINT_POSITION = [0.10, 1.47, 0.16, 0.0, 2.22, -1.9, -0.48, -1.39]
DISTANCE_THRESHOLD = 0.05

# These settings generate 20 grasp positions
L_ANGLE = 0
R_ANGLE = 358
T_ANGLE = 0
D_ANGLE = 358
RL_STEP = 30
TD_STEP = 30

processed_tomatoes = []

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
    return poses

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

def getNewValidTomato(lt):
    index = 0
    while index < len(lt) and checkDistanceUnderThreshold(lt[index]):
        index += 1

    if index == len(lt):
        rospy.logwarn("All reachable tomatoes have been picked")
        rospy.signal_shutdown("FINISHED TOMATOES")
        sys.exit()

    return lt[index]

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
    # gripper_client.wait_for_result()

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

def setTargetTomato(goal_pose, radius):
    """
    Disables the collisions in an area defined by a sphere.

    This sphere has a radius of AVOID_COLLISION_SPHERE_RADIUS.
    This in order to allow the approach of the arm to the tomato.

    :param goal_pose Pose: Position of the tomato to reach
    """
    scene.add_sphere("target_tomato", goal_pose, radius)
    scene.add_sphere("noCollisions", goal_pose, AVOID_COLLISION_SPHERE_RADIUS)
    diff_scene = PlanningScene()
    diff_scene.is_diff = True
    acm = scene.get_planning_scene(
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
    for elem in acm.entry_values:
        elem.enabled.extend([False, True])
    acm.entry_names.extend(["target_tomato", "noCollisions"])
    acm.entry_values.extend([
        AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)),
        AllowedCollisionEntry(enabled=[True] * len(acm.entry_names))
    ])
    acm.entry_values[-2].enabled[-1] = True
    diff_scene.allowed_collision_matrix = acm
    scene.apply_planning_scene(diff_scene)
    rospy.loginfo("Added sphere")

# def setTargetTomato(goal_pose, radius):
#     """
#     Disables the collisions in an area defined by a sphere.
#
#     This sphere has a radius of AVOID_COLLISION_SPHERE_RADIUS.
#     This in order to allow the approach of the arm to the tomato.
#
#     :param goal_pose Pose: Position of the tomato to reach
#     """
#     scene.add_sphere("noCollisions", goal_pose, AVOID_COLLISION_SPHERE_RADIUS)
#     diff_scene = PlanningScene()
#     diff_scene.is_diff = True
#     acm = scene.get_planning_scene(
#         PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
#     for elem in acm.entry_values:
#         elem.enabled.extend([True])
#     acm.entry_names.extend(["noCollisions"])
#     acm.entry_values.extend([
#         AllowedCollisionEntry(enabled=[True] * len(acm.entry_names))
#     ])
#     diff_scene.allowed_collision_matrix = acm
#     scene.apply_planning_scene(diff_scene)
#
#     lookAtTomato(goal_pose)
#     rospy.sleep(3)
#     scene.add_sphere("target_tomato", goal_pose, radius)
#     diff_scene = PlanningScene()
#     diff_scene.is_diff = True
#     acm = scene.get_planning_scene(
#         PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
#     for elem in acm.entry_values:
#         elem.enabled.extend([False])
#     acm.entry_names.extend(["target_tomato"])
#     acm.entry_values.extend([
#         AllowedCollisionEntry(enabled=[False] * len(acm.entry_names))
#     ])
#     acm.entry_values[-2].enabled[-1] = True
#
#     rospy.loginfo("Added sphere")


def removeTargetTomato():
    """Remove the previously set sphere from the scene."""
    scene.remove_world_object("target_tomato")
    scene.remove_world_object("noCollisions")
    diff_scene = PlanningScene()
    diff_scene.is_diff = True
    acm = scene.get_planning_scene(
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
    for elem in acm.entry_values:
        elem.enabled.pop()
        elem.enabled.pop()
    acm.entry_values.pop()
    acm.entry_values.pop()
    acm.entry_names.pop()
    acm.entry_names.pop()
    diff_scene.allowed_collision_matrix = acm
    scene.apply_planning_scene(diff_scene)
    rospy.loginfo("Removed sphere")


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
    if type(v0) is Point():
        v0 = [v0.x, v0.y, v0.z]
    if type(v1) is Point():
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


def create_grasp(pose, grasp_id):
    """
    :type pose: pose
        pose of the gripper for the grasp
    :type grasp_id: str
        name for the grasp
    :rtype: Grasp
    """
    g = Grasp()
    g.id = grasp_id
    pre_grasp_posture = JointTrajectory()
    # pre_grasp_posture.header.frame_id = "arm_tool_link"
    pre_grasp_posture.joint_names = [
        "gripper_left_finger_joint", "gripper_right_finger_joint"]
    jtpoint = JointTrajectoryPoint()
    jtpoint.positions = [0.044, 0.044]
    jtpoint.time_from_start = rospy.Duration(1)  # TODO check
    pre_grasp_posture.points.append(jtpoint)

    grasp_posture = copy.deepcopy(pre_grasp_posture)
    grasp_posture.points[0].time_from_start = rospy.Duration(1 + 1)
    jtpoint2 = JointTrajectoryPoint()
    jtpoint2.positions = [0.01, 0.01]
    jtpoint2.time_from_start = rospy.Duration(1 + 1 + 1)
    grasp_posture.points.append(jtpoint2)

    g.pre_grasp_posture = pre_grasp_posture
    g.grasp_posture = grasp_posture

    header = Header()
    header.frame_id = "base_footprint"
    q = [pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w]
    # Fix orientation from gripper_link to parent_link (tool_link)
    fix_tool_to_gripper_rotation_q = quaternion_from_euler(
        math.radians(0),
        math.radians(0),
        math.radians(0)
    )
    q = quaternion_multiply(q, fix_tool_to_gripper_rotation_q)
    fixed_pose = copy.deepcopy(pose)
    fixed_pose.orientation = Quaternion(*q)

    g.grasp_pose = PoseStamped(header, fixed_pose)
    g.grasp_quality = 0.1

    g.pre_grasp_approach = GripperTranslation()
    g.pre_grasp_approach.direction.vector.x = 1.0
    g.pre_grasp_approach.direction.vector.y = 0
    g.pre_grasp_approach.direction.vector.z = 0
    g.pre_grasp_approach.direction.header.frame_id = "arm_tool_link"
    g.pre_grasp_approach.desired_distance = 0.09
    g.pre_grasp_approach.min_distance = 0
    g.post_grasp_retreat = GripperTranslation()
    g.post_grasp_retreat.direction.vector.x = -1
    g.post_grasp_retreat.direction.vector.y = 0
    g.post_grasp_retreat.direction.vector.z = 0
    g.post_grasp_retreat.direction.header.frame_id = "arm_tool_link"
    g.post_grasp_retreat.desired_distance = 0.09
    g.post_grasp_retreat.min_distance = 0

    g.max_contact_force = 0
    g.allowed_touch_objects = ""

    return g


def pickTomato(goal_pose):
    gpos = generate_grasp_poses(goal_pose, radius=TARGET_OFFSET)

    grasps = []
    for idx, pose in enumerate(gpos):
        grasps.append(
            create_grasp(pose, "grasp_" + str(idx)))

    move_group.pick("target_tomato", grasp=grasps)


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("positionReacher")

robot = moveit_commander.RobotCommander()
names = robot.get_group_names()
scene = moveit_commander.PlanningSceneInterface()

gripper_client = actionlib.SimpleActionClient(
    "/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
gripper_client.wait_for_server()
rospy.wait_for_service("/tomato_vision_manager/tomato_position_service")
getTomatoPosesProxy = rospy.ServiceProxy(
    "/tomato_vision_manager/tomato_position_service", LatestTomatoPositions)
pickup = SimpleActionClient('/pickup', PickupAction)
pickup.wait_for_server()
point_head_client = actionlib.SimpleActionClient(
    "/head_controller/point_head_action", PointHeadAction)
point_head_client.wait_for_server()
scene.remove_attached_object(name="target_tomato")
move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)
move_group.set_planner_id("RRTstarkDefaultConfig")
move_group.set_planning_time(10)

tomato_poses = getTomatoPoses()
goal_pose = getNewValidTomato(tomato_poses)

setTargetTomato(goal_pose, 0.03)
pickTomato(goal_pose)
move_group.set_joint_value_target(BASKET_JOINT_POSITION)
move_group.go(wait=True)
openGripper()
scene.remove_attached_object(name="target_tomato")
removeTargetTomato()
