import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import sys
import rospy
import tf
import copy
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import actionlib

sub = None

def poseCallBack(positions):
    toReach = copy.deepcopy(positions.poses)
    print(toReach)
    sub.unregister()
    client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
    for pose in toReach:

        rospy.loginfo("Going to tomato %d", pose.orientation.y)
        goal_pose = Pose()
        goal_pose.position.x = 0.40#pose.position.x - 0.20
        goal_pose.position.y = pose.position.y
        goal_pose.position.z = pose.position.z
        goal_pose.orientation.w = 1
        move_group.set_pose_target(goal_pose)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        rospy.loginfo("Tomato Reached")

        rospy.loginfo("Going Home")
        goal = PlayMotionGoal()
        goal.motion_name = "home"
        goal.skip_planning = False
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(15.0))
        rospy.loginfo("Home reached")


GROUP_NAME = "arm_torso"

# radiants
# goal_pose.pose.orientation = tf.transformations.from_quaternion_euler(0, 0, 0)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("positionReacher")

sub = rospy.Subscriber("/tomato_vision_manager/tomato_position",
                       PoseArray, poseCallBack)


robot = moveit_commander.RobotCommander()
names = robot.get_group_names()
print(names)

scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)
# display_trajectory_publisher = rospy.Publisher(
#     "/move_group/display_planned_path",
#     moveit_msgs.msg.DisplayTrajectory,
#     queue_size=20,
# )


rospy.spin()
