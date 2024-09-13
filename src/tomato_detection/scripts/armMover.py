#!/usr/bin/env python

import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, PlanningSceneComponents
import sys
import rospy
import tf
import copy
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import actionlib
from visualization_msgs.msg import Marker
import math
from enum import Enum


GROUP_NAME = "arm_torso"
TARGET_OFFSET = 0.18
APPROACH_OFFSET = 0.28
AVOID_COLLISION_SPHERE_RAIDUS = 0.10
EFFORT = 0.3

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
gripper_pub = rospy.Publisher(
    "/parallel_gripper_controller/command", JointTrajectory, queue_size=2)


def closeGripper(tomato_radius):
    """
    Close gripper.

    :param float tomato_radius: Radius of the tomato to grab.
    """
    command = JointTrajectory()
    command.joint_names = ["gripper_left_finger_joint",
                           "gripper_right_finger_joint"]

    point = JointTrajectoryPoint()
    tomato_radius -= 0.01
    point.positions = [tomato_radius, tomato_radius]
    point.effort = [EFFORT, EFFORT]
    point.time_from_start = rospy.Duration(0.5)

    command.points.append(point)
    gripper_pub.publish(command)


def openGripper():
    """Open gripper."""
    command = JointTrajectory()
    command.joint_names = ["gripper_left_finger_joint",
                           "gripper_right_finger_joint"]

    point = JointTrajectoryPoint()
    point.positions = [0.1, 0.1]
    point.effort = [EFFORT, EFFORT]
    point.time_from_start = rospy.Duration(0.5)

    command.points.append(point)
    gripper_pub.publish(command)


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


def disableCollisionsAtTarget(goal_pose):
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


def pickTomato(tomato_id, goal_pose, radius):
    """
    State machine that enables the picking of the tomatoes.

    :param tomato_id Int: Id of the tomato to pick
    :param goal_pose Pose: Position of the tomato wrt the base_footprint frame
    :param radius Double: Radius of the tomato to pick
    """
    state = States.PLAN_APPROACH
    old_state = state
    actionlib_client = actionlib.SimpleActionClient(
        "play_motion", PlayMotionAction)

    if math.isnan(float(goal_pose.pose.position.x)):
        rospy.logerr("Nan received")
        return

    while True:
        if state == States.HOME:
            return

        elif state == States.PLAN_APPROACH:
            disableCollisionsAtTarget(goal_pose)
            old_state = state
            rospy.loginfo("Planning approach for tomato [%d]", tomato_id)
            goal_pose.pose.position.x -= APPROACH_OFFSET
            move_group.set_pose_target(goal_pose)
            success = move_group.go(wait=True)
            goal_pose.pose.position.x += APPROACH_OFFSET
            if success:
                rospy.loginfo("Planning approach SUCCESS")
                state = States.EXECUTING_MOVEMENT
            else:
                rospy.logwarn("Planning approach FAIL")
                break

        elif state == States.PLAN_PICK:
            rospy.sleep(1.0)
            old_state = state
            rospy.loginfo("Planning pick for tomato [%d]", tomato_id)
            goal_pose.pose.position.x -= TARGET_OFFSET
            target = [goal_pose.pose]
            (path, frac) = move_group.compute_cartesian_path(target, 0.01, 0)
            move_group.set_pose_target(goal_pose)
            success = move_group.execute(path, wait=True)
            goal_pose.pose.position.x += TARGET_OFFSET
            if success:
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
            # TODO how to check if grab was successful
            state = States.HOME

        elif state == States.EXECUTING_MOVEMENT:
            rospy.loginfo("Executing movement")
            move_group.stop()
            move_group.clear_pose_targets()
            state = States(old_state.value + 1)


def poseCallBack(positions):
    """
    Ros callback for "/tomato_vision_manager/tomato_position" topic.

    :param positions PoseArray: Positions of all the tomatoes with additional
    information in orientation
    """
    toReachTS = copy.deepcopy(positions.poses)
    # print(toReach)
    sub.unregister()

    toReach = sorted(toReachTS, key=lambda elem: elem.position.x)

    for pose in toReach:
        if math.isnan(float(pose.position.x)):
            rospy.logerr("Nan received")
            continue

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

        pickTomato(pose.orientation.y, goal_pose, pose.orientation.z)


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

print(move_group.get_planning_frame())
print(move_group.get_end_effector_link())
# display_trajectory_publisher = rospy.Publisher(
#     "/move_group/display_planned_path",
#     moveit_msgs.msg.DisplayTrajectory,
#     queue_size=20,
# )


rospy.spin()


# TODO s
# - remove obstacle avoidance only when doing the cartesian movement
# - Increase obstacle avoidance area
# - Finda a way to know the current effort in order to determine
#   if the grab was succesful or not
# - Plan whiel doing the previous movement
