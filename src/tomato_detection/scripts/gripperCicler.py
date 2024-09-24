import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

gripper_pub = rospy.Publisher(
    "/parallel_gripper_left_controller/command", JointTrajectory, queue_size=2)


def closeGripper():
    command = JointTrajectory()
    command.joint_names = ["gripper_left_left_finger_joint", "gripper_left_right_finger_joint"]

    point = JointTrajectoryPoint()
    point.positions = [0, 0]
    point.effort = [1, 1]
    point.time_from_start = rospy.Duration(1.0)

    command.points.append(point)
    gripper_pub.publish(command)


def openGripper():
    command = JointTrajectory()
    command.joint_names = ["gripper_left_left_finger_joint", "gripper_left_right_finger_joint"]

    point = JointTrajectoryPoint()
    point.positions = [1, 1]
    point.effort = [1, 1]
    point.time_from_start = rospy.Duration(1.0)

    command.points.append(point)
    gripper_pub.publish(command)


rospy.init_node("gripperTester")
closeGripper()
rospy.sleep(3)
openGripper()
rospy.spin()
