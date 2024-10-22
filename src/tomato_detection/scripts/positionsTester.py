import rospy
import numpy as np
import math
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import quaternion_multiply, euler_from_quaternion
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import PlanningScene
import moveit_commander
import sys

APPROACH_OFFSET = 0.30
L_ANGLE = 60
R_ANGLE = 60
T_ANGLE = 0
D_ANGLE = 60


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
    for altitude in range(180 - L_ANGLE, 180 + R_ANGLE + 1, 30):  # NOQA
        altitude = math.radians(altitude)
        # azimuth is pitch
        for azimuth in range(T_ANGLE, D_ANGLE + 1, 20):  # NOQA
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


rospy.init_node("testPositions")
moveit_commander.roscpp_initialize(sys.argv)

goal_pose = PoseStamped()
goal_pose.header.frame_id = "base_footprint"
goal_pose.pose.position.x = 0.8
goal_pose.pose.position.y = 0
goal_pose.pose.position.z = 0.8
goal_pose.pose.orientation.x = 0
goal_pose.pose.orientation.y = 0
goal_pose.pose.orientation.z = 0
goal_pose.pose.orientation.w = 1

poses = generate_grasp_poses(goal_pose)

scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm_torso")
move_group.set_planner_id("SPARS2")

print(len(poses))

for pose in poses:
    move_group.set_pose_target(pose)
    move_group.go(wait=True)
