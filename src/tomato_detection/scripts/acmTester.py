from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, PlanningSceneComponents
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
import rospy

AVOID_COLLISION_SPHERE_RAIDUS = 0.1

def disableCollisionsAtTarget(goal_pose):
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

rospy.init_node("testACM")
moveit_commander.roscpp_initialize(sys.argv)
scene = moveit_commander.PlanningSceneInterface()

gp = PoseStamped()
gp.header.frame_id = "base_footprint"
gp.pose.position.x = 0.7
gp.pose.position.y = 0.0
gp.pose.position.z = 0.8
gp.pose.orientation.x = 0
gp.pose.orientation.y = 0
gp.pose.orientation.z = 0
gp.pose.orientation.w = 1

disableCollisionsAtTarget(gp)
rospy.sleep(10.0)
acm = scene.get_planning_scene(
    PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
print(acm)
removeSphere()
