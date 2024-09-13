import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry, PlanningSceneComponents
import rospy
import sys

GROUP_NAME = "arm_torso"

goal_pose = PoseStamped()
goal_pose.header.frame_id = "base_footprint"
goal_pose.pose.position.x = 0.94
goal_pose.pose.position.y = 0.13
goal_pose.pose.position.z = 1.34

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("positionReacher")

robot = moveit_commander.RobotCommander()
names = robot.get_group_names()

scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)

scene.add_sphere("targetTomato", goal_pose, radius=0.1)

diff_scene = PlanningScene()
diff_scene.is_diff = True
acm = scene.get_planning_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
print("Names: ", acm.entry_names)
# print(acm.entry_values)
for elem in acm.entry_values:
    elem.enabled.append(True)
acm.entry_names.append("targetTomato")
acm.entry_values.append(AllowedCollisionEntry(enabled=[True] * len(acm.entry_names)))
diff_scene.allowed_collision_matrix = acm
scene.apply_planning_scene(diff_scene)
newAcm = scene.get_planning_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
print("New Names: ", newAcm.entry_names)
rospy.spin()
