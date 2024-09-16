from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import rospy
import actionlib

rospy.init_node("gripper_closer")

client = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
client.wait_for_server()
rospy.loginfo("Server ready")

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]

point = JointTrajectoryPoint()
point.positions = [0.05, 0.05]
point.time_from_start = rospy.Duration(1.0)
goal.trajectory.points.append(point)

client.send_goal(goal)
rospy.loginfo("goal sent")
res = client.wait_for_result()
rospy.loginfo("result")
print(res)

rospy.spin()
