import rospy
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib

best_head_tilt = 0.1

def resetHead():
    head_goal = FollowJointTrajectoryGoal()
    look_point = JointTrajectoryPoint()
    look_point.positions = [0.0, best_head_tilt]
    look_point.velocities = [0.0, 0.0]
    look_point.time_from_start = rospy.Duration(3.0)
    head_goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
    head_goal.trajectory.points = [look_point]
    rospy.loginfo("Resetting head position")
    head_client.send_goal(head_goal)
    head_client.wait_for_result()
    rospy.loginfo("Head resetted")

rospy.init_node("head_tester")

point_head_client = actionlib.SimpleActionClient("/head_controller/point_head_action", PointHeadAction)
point_head_client.wait_for_server()


tomato_point = PointStamped()
tomato_point.point.x = 0.3
tomato_point.point.y = 0.3
tomato_point.point.z = 1
tomato_point.header.frame_id = "/xtion_rgb_optical_frame"

head_goal = PointHeadGoal()
head_goal.pointing_frame = "/xtion_rgb_optical_frame"
head_goal.pointing_axis.x = 0
head_goal.pointing_axis.y = 0
head_goal.pointing_axis.z = 1
head_goal.min_duration = rospy.Duration(0.1)
head_goal.max_velocity = 0.25
head_goal.target = tomato_point
rospy.loginfo("wait for result")
point_head_client.send_goal(head_goal)
point_head_client.wait_for_result()
rospy.loginfo("DONE")

head_client = actionlib.SimpleActionClient("/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
head_client.wait_for_server()
resetHead()

rospy.spin()
