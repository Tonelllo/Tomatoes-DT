#!/usr/bin/env python

import moveit_commander
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, PlanningSceneComponents
import sys
import rospy
import copy
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
from visualization_msgs.msg import Marker
import math
from enum import Enum


GROUP_NAME = "arm_torso"
TARGET_OFFSET = 0.20
APPROACH_OFFSET = 0.35
AVOID_COLLISION_SPHERE_RAIDUS = 0.10
EFFORT = 0.3
CARTESIAN_FAILURE_THRESHOLD = 0.9
OPEN_GRIPPER_POS = 0.05

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
gripper_pub = rospy.Publisher(
    "/parallel_gripper_controller/command", JointTrajectory, queue_size=2)


def closeGripper(tomato_radius):
    """
    Close gripper.

    :param float tomato_radius: Radius of the tomato to grab.
    """
    gripper_client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "gripper_left_finger_joint", "gripper_right_finger_joint"]

    tomato_radius -= 0.01

    point = JointTrajectoryPoint()
    point.positions = [tomato_radius, tomato_radius]
    point.time_from_start = rospy.Duration(1.0)
    goal.trajectory.points.append(point)

    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


def openGripper():
    """Open gripper."""
    gripper_client.wait_for_server()

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


def disableCollisionsAtTarget(goal_pose):
    """
    Disables the collisions in an area defined by a sphere.

    This sphere has a radius of AVOID_COLLISION_SPHERE_RAIDUS.
    This in order to allow the approach of the arm to the tomato.

    :param goal_pose Pose: Position of the tomato to reach
    """
    scene.add_sphere("targetTomato", goal_pose,
                     radius=AVOID_COLLISION_SPHERE_RAIDUS)
    diff_scene = PlanningScene()
    diff_scene.is_diff = True
    acm = scene.get_planning_scene(
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
    for elem in acm.entry_values:
        elem.enabled.append(True)
    acm.entry_names.append("targetTomato")
    acm.entry_values.append(AllowedCollisionEntry(
        enabled=[True] * len(acm.entry_names)))
    diff_scene.allowed_collision_matrix = acm
    scene.apply_planning_scene(diff_scene)


def removeSphere():
    """Remove the previously set sphere from the scene."""
    scene.remove_world_object("targetTomato")
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


class States(Enum):
    """States for the state machine."""

    PLAN_APPROACH = 1
    PLAN_PICK = 2
    PLAN_GRAB = 3
    PLAN_BACK = 4
    PLAN_HOME = 5
    PLAN_RELEASE = 6
    HOME = 7
    EXECUTING_MOVEMENT = 8


def positionCalculator():
    """Calculates different poses to reach the tomato."""
    pass


def pickTomato(tomato_id, goal_pose, radius):
    """
    State machine that enables the picking of the tomatoes.

    :param tomato_id Int: Id of the tomato to pick
    :param goal_pose Pose: Position of the tomato wrt the base_footprint frame
    :param radius Double: Radius of the tomato to pick
    """
    failed_pick = False
    state = States.PLAN_APPROACH
    old_state = state
    actionlib_client = actionlib.SimpleActionClient(
        "play_motion", PlayMotionAction)

    if math.isnan(float(goal_pose.pose.position.x)):
        rospy.logerr("Nan received")
        return "nan"

    while True:
        if state == States.HOME:
            return "home"

        elif state == States.PLAN_APPROACH:
            old_state = state
            rospy.loginfo("Planning approach for tomato [%d]", tomato_id)
            goal_pose.pose.position.x -= APPROACH_OFFSET
            move_group.set_pose_target(goal_pose)
            plan1 = move_group.plan()
            alt_pose = copy.deepcopy(goal_pose)
            alt_pose.pose.orientation.x = -0.7071068
            alt_pose.pose.orientation.y = 0
            move_group.set_pose_target(alt_pose)
            plan2 = move_group.plan()
            (success1, trajectory1, time1, error1) = plan1
            (success2, trajectory2, time2, error2) = plan2

            if not (success1 or success2):
                rospy.loginfo("Planning trajectories for approach FAILED")
                return "plan_fail"

            l1 = getTrajectoryLen(trajectory1)
            l2 = getTrajectoryLen(trajectory2)

            if l1 is None and l2 is not None:
                goal_pose = alt_pose
                success = move_group.execute(trajectory2, wait=True)
                rospy.loginfo("Arm reversed")
            elif l2 is None and l1 is not None:
                success = move_group.execute(trajectory1, wait=True)
                rospy.loginfo("Arm normal")
            elif l1 is None and l2 is None:
                return "plan_fail"


            if l2 < l1:
                goal_pose = alt_pose
                success = move_group.execute(trajectory2, wait=True)
                rospy.loginfo("Arm reversed")
            else:
                success = move_group.execute(trajectory1, wait=True)
                rospy.loginfo("Arm normal")

            if success1 or success2:
                rospy.loginfo("Planning approach SUCCESS")
                state = States.EXECUTING_MOVEMENT
            else:
                rospy.logwarn("Planning approach FAIL")
                return "plan_fail"

            goal_pose.pose.position.x += APPROACH_OFFSET

        elif state == States.PLAN_PICK:
            disableCollisionsAtTarget(goal_pose)
            rospy.sleep(1.0)
            old_state = state
            rospy.loginfo("Planning pick for tomato [%d]", tomato_id)
            goal_pose.pose.position.x -= TARGET_OFFSET
            target = [goal_pose.pose]
            (path, frac) = move_group.compute_cartesian_path(target, 0.01, 0)
            move_group.set_pose_target(goal_pose)
            success = move_group.execute(path, wait=True)
            goal_pose.pose.position.x += TARGET_OFFSET
            if frac <= CARTESIAN_FAILURE_THRESHOLD:
                failed_pick = True

            if success and frac > CARTESIAN_FAILURE_THRESHOLD:
                rospy.loginfo("Planning pick SUCCESS")
                state = States.EXECUTING_MOVEMENT
            else:
                rospy.logwarn("Planning pick FAIL")
                state = States.PLAN_HOME

        elif state == States.PLAN_GRAB:
            old_state = state
            rospy.loginfo("Planning grab for tomato [%d]", tomato_id)
            closeGripper(radius)
            rospy.sleep(1)
            # TODO how to check if grab was successful
            state = States.PLAN_BACK

        elif state == States.PLAN_BACK:
            old_state = state
            rospy.loginfo("Plannig back")
            goal_pose.pose.position.x -= APPROACH_OFFSET
            target = [goal_pose.pose]
            (path, frac) = move_group.compute_cartesian_path(target, 0.01, 0)
            move_group.set_pose_target(goal_pose)
            success = move_group.execute(path, wait=True)
            goal_pose.pose.position.x += APPROACH_OFFSET

            # Here in any case you go back home
            if success:
                rospy.loginfo("Planning back SUCCESS")
                state = States.EXECUTING_MOVEMENT
            else:
                rospy.logwarn("Planning back FAIL")
                state = States.PLAN_HOME

        elif state == States.PLAN_HOME:
            removeSphere()
            old_state = state
            rospy.loginfo("Planning approach for HOME")
            goal = PlayMotionGoal()
            goal.motion_name = "home"
            goal.skip_planning = False
            actionlib_client.send_goal(goal)
            actionlib_client.wait_for_result(rospy.Duration(15.0))
            rospy.loginfo("HOME reached")
            # TODO What if it fails?
            state = States.PLAN_RELEASE

        elif state == States.PLAN_RELEASE:
            old_state = state
            rospy.loginfo("Planning release for tomato [%d]", tomato_id)
            openGripper()
            rospy.sleep(1)
            if failed_pick:
                return "plan_fail"
            state = States.HOME

        elif state == States.EXECUTING_MOVEMENT:
            rospy.loginfo("Executing movement")
            move_group.stop()
            move_group.clear_pose_targets()
            state = States(old_state.value + 1)


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

        setMarker(goal_pose.pose)
        print(goal_pose.pose.position)

        rospy.loginfo("Going to tomato %d", pose.orientation.y)

        ret = pickTomato(pose.orientation.y, goal_pose, pose.orientation.z / 2)
        if ret in ["plan_fail", "nan"]:
            last_tomatoes.append(int(pose.orientation.y))
    else:
        rospy.error("Received NaN")

    sub = rospy.Subscriber("/tomato_vision_manager/tomato_position",
                           PoseArray, poseCallBack)


# radiants
# goal_pose.pose.orientation = tf.transformations.from_quaternion_euler(0, 0, 0)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("positionReacher")


sub = rospy.Subscriber("/tomato_vision_manager/tomato_position",
                       PoseArray, poseCallBack)

robot = moveit_commander.RobotCommander()
names = robot.get_group_names()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)
# move_group.set_planner_id("RRTstar") # TODO does nothing
gripper_client = actionlib.SimpleActionClient(
    "/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

addBasket()

assert move_group.get_planning_frame() == "base_footprint", "Wrong planning frame"

rospy.spin()

# TODO s
# - Plan while doing the previous movement
