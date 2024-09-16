#!/usr/bin/env python

import moveit_commander
from geometry_msgs.msg import PoseStamped
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




GROUP_NAME = "arm_torso"
TARGET_OFFSET = 0.18
APPROACH_OFFSET = 0.28
AVOID_COLLISION_SPHERE_RADIUS = 0.10
EFFORT = 0.3
CARTESIAN_FAILURE_THRESHOLD = 0.9

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
gripper_pub = rospy.Publisher(
    "/parallel_gripper_controller/command", JointTrajectory, queue_size=2)


def addBasket():
    """Add the tomato basket to the scene."""
    box_position = PoseStamped()
    box_position.header.frame_id = "base_footprint"
    box_position.pose.position.x = 0.4
    box_position.pose.position.y = 0
    box_position.pose.position.z = 0.1
    box_position.pose.orientation.w = 1.0
    scene.add_box("tomato_basket", box_position, (0.20, 0.40, 0.20))


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
        elem.enabled.append(True)
    acm.entry_names.append("noCollisions")
    acm.entry_values.append(AllowedCollisionEntry(
        enabled=[True] * len(acm.entry_names)))
    diff_scene.allowed_collision_matrix = acm
    scene.apply_planning_scene(diff_scene)
    rospy.loginfo("Added sphere")


def removeTargetTomato():
    """Remove the previously set sphere from the scene."""
    scene.remove_world_object("target_tomato")
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
    rospy.loginfo("Removed sphere")


last_tomatoes = []


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

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def quaternion_from_vectors(v0, v1):
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

def generate_grasp_poses(object_pose):
    # Compute all the points of the sphere with step X
    # http://math.stackexchange.com/questions/264686/how-to-find-the-3d-coordinates-on-a-celestial-spheres-surface
    radius = TARGET_OFFSET
    ori_x = 0.0
    ori_y = 0.0
    ori_z = 0.0
    sphere_poses = []
    rotated_q = quaternion_from_euler(0.0, 0.0, math.radians(180))

    # altitude is yaw
    for altitude in range(120, 240, 20):  # NOQA
        altitude = math.radians(altitude)
        # azimuth is pitch
        for azimuth in range(-60, 60, 20):  # NOQA
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
            q = quaternion_from_euler(math.radians(0.0), pitch, yaw)

            x += object_pose.pose.position.x
            y += object_pose.pose.position.y
            z += object_pose.pose.position.z
            current_pose = Pose(
                Point(x, y, z), Quaternion(*q))
            sphere_poses.append(current_pose)
    return sphere_poses


def create_grasp(pose, grasp_id):
    """
    :type pose: Pose
        pose of the gripper for the grasp
    :type grasp_id: str
        name for the grasp
    :rtype: Grasp
    """
    g = Grasp()
    g.id = grasp_id

    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "arm_tool_link"
    pre_grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    jtpoint = JointTrajectoryPoint()
    jtpoint.positions = [0.05, 0.05]
    jtpoint.time_from_start = rospy.Duration(2) # TODO check
    pre_grasp_posture.points.append(jtpoint)

    grasp_posture = copy.deepcopy(pre_grasp_posture)
    grasp_posture.points[0].time_from_start = rospy.Duration(2 + 2)
    jtpoint2 = JointTrajectoryPoint()
    jtpoint2.positions = [0.01, 0.01]
    jtpoint2.time_from_start = rospy.Duration(2 + 2 + 2)
    grasp_posture.points.append(jtpoint2)

    g.pre_grasp_posture = pre_grasp_posture
    g.grasp_posture = grasp_posture

    header = Header()
    header.frame_id = "base_footprint"
    q = [pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w]
    # Fix orientation from gripper_link to parent_link (tool_link)
    fix_tool_to_gripper_rotation_q = quaternion_from_euler(
        math.radians(90),
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
    g.pre_grasp_approach.desired_distance = TARGET_OFFSET
    g.pre_grasp_approach.min_distance = 0
    g.post_grasp_retreat = GripperTranslation()
    g.post_grasp_retreat.direction.vector.x = -1
    g.post_grasp_retreat.direction.vector.y = 0
    g.post_grasp_retreat.direction.vector.z = 0
    g.post_grasp_retreat.direction.header.frame_id = "arm_tool_link"
    g.post_grasp_retreat.desired_distance = TARGET_OFFSET
    g.post_grasp_retreat.min_distance = 0

    g.max_contact_force = 0
    g.allowed_touch_objects = ""

    return g

def create_grasps_from_poses(sphere_poses):
    """
    :type sphere_poses: []
        [] of Pose
    """
    grasps = []
    for idx, pose in enumerate(sphere_poses):
        grasps.append(
            create_grasp(pose, "grasp_" + str(idx)))
    return grasps

def createGripperTranslation(direction_vector,
                                desired_distance=0.15,
                                min_distance=0.01):
    """Returns a GripperTranslation message with the
        direction_vector and desired_distance and min_distance in it.
    Intended to be used to fill the pre_grasp_approach
        and post_grasp_retreat field in the Grasp message."""
    g_trans = GripperTranslation()
    g_trans.direction.header.frame_id = "arm_tool_link"
    g_trans.direction.header.stamp = rospy.Time.now()
    g_trans.direction.vector.x = direction_vector.x
    g_trans.direction.vector.y = direction_vector.y
    g_trans.direction.vector.z = direction_vector.z
    g_trans.desired_distance = desired_distance
    g_trans.min_distance = min_distance
    return g_trans

def create_placings_from_object_pose(posestamped):
    """ Create a list of PlaceLocation of the object rotated every 15deg"""
    place_locs = []
    pre_grasp_posture = JointTrajectory()
    # Actually ignored....
    pre_grasp_posture.header.frame_id = "base_footprint"
    pre_grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    jtpoint = JointTrajectoryPoint()
    jtpoint.positions = [0.05, 0.05]
    #Let enough time for the gripper to release the object
    jtpoint.time_from_start = rospy.Duration(3.5)
    pre_grasp_posture.points.append(jtpoint)
    # Generate all the orientations every step_degrees_yaw deg
    for yaw_angle in np.arange(0.0, 2.0 * math.pi, math.radians(10)):
        pl = PlaceLocation()
        pl.place_pose = posestamped
        newquat = quaternion_from_euler(0.0, 0.0, yaw_angle)
        pl.place_pose.pose.orientation = Quaternion(
            newquat[0], newquat[1], newquat[2], newquat[3])
        # TODO: the frame is ignored, this will always be the frame of the gripper
        # so arm_tool_link
        pl.pre_place_approach = createGripperTranslation(
            Vector3(1.0, 0.0, 0.0))
        pl.post_place_retreat = createGripperTranslation(
            Vector3(-1.0, 0.0, 0.0))

        pl.post_place_posture = pre_grasp_posture
        place_locs.append(pl)

    return place_locs

def planPick(goal_pose, tomato_radius):
    actionlib_client = SimpleActionClient(
        "play_motion", PlayMotionAction)
    sphere_poses = generate_grasp_poses(goal_pose)
    grasps = create_grasps_from_poses(sphere_poses)

    pg = PickupGoal()
    pg.target_name = "target_tomato"
    pg.group_name = "arm_torso"
    pg.possible_grasps = grasps
    pg.allowed_planning_time = 35.0
    pg.planning_options.planning_scene_diff.is_diff = True
    pg.planning_options.planning_scene_diff.robot_state.is_diff = True
    pg.planning_options.plan_only = False
    pg.planning_options.replan = True
    pg.planning_options.replan_attempts = 3  # 10
    pg.allowed_touch_objects = []
    pg.attached_object_touch_links = ['<octomap>', "noCollisions"]
    links_to_allow_contact = ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]
    pg.attached_object_touch_links.extend(links_to_allow_contact)

    pick_client.send_goal(pg)
    pick_client.wait_for_result()
    result = pick_client.get_result()
    if result.error_code.val != 1:
        rospy.logerr("Failed to find valid grasp for tomato")
        # return False
        # goal = PlayMotionGoal()
        # goal.motion_name = "home"
        # goal.skip_planning = False
        # actionlib_client.send_goal(goal)
        # actionlib_client.wait_for_result(rospy.Duration(15.0))


    place_targ = PoseStamped()
    place_targ.header.frame_id = "base_footprint"
    place_targ.pose.position.x = 0.6
    place_targ.pose.position.y = 0.5
    place_targ.pose.position.z = 0.7
    place_targ.pose.orientation.w = 1

    placeg = PlaceGoal()
    placeg.group_name = "arm_torso"
    placeg.attached_object_name = "target_tomato"
    place_locations = create_placings_from_object_pose(place_targ)
    placeg.place_locations = place_locations
    placeg.allowed_planning_time = 15.0
    placeg.planning_options.planning_scene_diff.is_diff = True
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
    placeg.planning_options.plan_only = False
    placeg.planning_options.replan = True
    placeg.planning_options.replan_attempts = 3
    placeg.allowed_touch_objects = ['<octomap>', "noCollisions"]
    placeg.allowed_touch_objects.extend(links_to_allow_contact)

    place_client.send_goal(placeg)

    place_client.wait_for_result()
    result = place_client.get_result()
    if result.error_code.val != 1:
        rospy.logerr("Failed to find place path")
        return False

    return True


def poseCallBack(positions):
    """
    Ros callback for "/tomato_vision_manager/tomato_position" topic.

    :param positions PoseArray: Positions of all the tomatoes with additional
    information in orientation
    """
    global sub
    toReachTS = copy.deepcopy(positions.poses)
    # print(toReach)
    sub.unregister()

    toReachTS = filter(isRipe, toReachTS)
    toReach = sorted(toReachTS, key=lambda elem: elem.position.x)

    index = 0
    while index < len(toReach) and int(toReach[index].orientation.y) in last_tomatoes:
        index += 1

    if index == len(toReach):
        rospy.logwarn("All reachable tomatoes have been picked")
        rospy.signal_shutdown("FINISHED TOMATOES")
        sys.exit()

    pose = toReach[index]
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

        print(goal_pose.pose.position)

        rospy.loginfo("Going to tomato %d", pose.orientation.y)

        # TODO pick
        setTargetTomato(goal_pose, pose.orientation.z / 2)
        res = planPick(goal_pose, pose.orientation.z / 2)
        # removeTargetTomato()
        rospy.sleep(3)
        if not res:
            last_tomatoes.append(int(pose.orientation.y))

    else:
        rospy.error("Received NaN")

    sub = rospy.Subscriber("/tomato_vision_manager/tomato_position",
                           PoseArray, poseCallBack)


# radiants
# goal_pose.pose.orientation = tf.transformations.from_quaternion_euler(0, 0, 0)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("positionReacher")

robot = moveit_commander.RobotCommander()
names = robot.get_group_names()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)

pick_client = SimpleActionClient("/pickup", PickupAction)
place_client = SimpleActionClient("/place", PlaceAction)
rospy.loginfo("Wating for action server")
pick_client.wait_for_server()
rospy.loginfo("Action server started")

sub = rospy.Subscriber("/tomato_vision_manager/tomato_position",
                       PoseArray, poseCallBack)

addBasket()

assert move_group.get_planning_frame() == "base_footprint", "Wrong planning frame"

rospy.spin()

# TODO s
# - Plan while doing the previous movement
