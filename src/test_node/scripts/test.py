import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
import multiprocessing

moveit_commander.roscpp_initialize(sys.argv)

def tuck_arm():
    client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
    client.wait_for_server()
    rospy.loginfo("...connected.")

    rospy.wait_for_message("joint_states", JointState)
    rospy.sleep(3.0)

    rospy.loginfo("Tuck arm...")
    goal = PlayMotionGoal()
    goal.motion_name = 'home'
    goal.skip_planning = False

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(20.0))
    rospy.loginfo("Arm tucked.")

GROUP_NAME = "arm_torso"

def planWorker(q, target_pose):
    rospy.loginfo("Planning")
    move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    move_group.set_pose_target(target_pose)
    q.put(move_group.plan())
    rospy.loginfo("Planned")




target_pose_1 = PoseStamped()
target_pose_1.header.frame_id = "base_footprint"
target_pose_1.pose.position.x = 0.8
target_pose_1.pose.position.y = 0
target_pose_1.pose.position.z = 0.9
target_pose_1.pose.orientation.x = 0.7
target_pose_1.pose.orientation.y = 0.0
target_pose_1.pose.orientation.z = 0.0
target_pose_1.pose.orientation.w = 0.7

target_pose_2 = PoseStamped()
target_pose_2.header.frame_id = "base_footprint"
target_pose_2.pose.position.x = 0.7
target_pose_2.pose.position.y = 0
target_pose_2.pose.position.z = 0.9
target_pose_2.pose.orientation.x = 0.7
target_pose_2.pose.orientation.y = 0.0
target_pose_2.pose.orientation.z = 0.0
target_pose_2.pose.orientation.w = 0.7

rospy.init_node("testpy")


move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)

que = multiprocessing.Queue()
# tuck_arm()

process1 = multiprocessing.Process(
    target=planWorker, args=(que, target_pose_1))
process2 = multiprocessing.Process(
    target=planWorker, args=(que, target_pose_2))

process1.start()
process2.start()

process1.join()
process2.join()

plan1 = que.get()
plan2 = que.get()

(success1, trajectory1, time1, error1) = plan1
(success2, trajectory2, time2, error2) = plan2


tuck_arm()
move_group.execute(trajectory1, wait=True)
move_group.stop()
move_group.clear_pose_targets()

tuck_arm()
move_group.execute(trajectory2, wait=True)
move_group.stop()
move_group.clear_pose_targets()
