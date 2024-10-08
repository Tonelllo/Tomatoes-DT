#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from control_msgs.msg import JointTrajectoryControllerState
from tomato_detection.srv import BestPos
from ultralytics import YOLO
import rospkg
from enum import Enum


class Model_used(Enum):
    TRACKING = 1
    DETECTION = 2


mode = Model_used.DETECTION

"""
Description:
    This script is used to calclulate the best position of the head of the robot,
    which means the position where there are more tomatoes and they are closest
    to the center. The heart of the calculation of the quality of the position
    is:
    113)  goodness_index = position_index + window_count * 5
    where 5 is a magic number.
"""

MAX_HISTORY = 30
bridge = CvBridge()

rospack = rospkg.RosPack()
node_path = rospack.get_path("tomato_detection") + "/models/"
model = YOLO(node_path + "yolov10_80p_70r.pt")
# pub = rospy.Publisher("detection/current_count", Int8, queue_size=5)
count = 0
history = []
camera_info = None
camera_center = None
head_subscriber = None
image_subscriber = None
best_tilt = 0

max_goodness = 0

print("STARTED")


def getCenter(x, y, w, h):
    return ((x+w) / 2, (y+h) / 2)


def giveBestPosition(req):
    global best_tilt, number_x, number_y, val_x, val_y, image_subscriber, head_subscriber

    if req.activate:
        image_subscriber = rospy.Subscriber(
            "/tomato_sync/image_rgb", Image, callback)
        head_subscriber = rospy.Subscriber("/head_controller/state",
                                           JointTrajectoryControllerState, positionGetter)
        return False, 0
    else:
        image_subscriber.unregister()
        head_subscriber.unregister()
        return True, best_tilt


def positionGetter(data):
    global head_tilt
    # Get the second joint position since
    # the head is only going up and down
    head_tilt = data.actual.positions[1]


def getParams(data):
    global camera_center, max_dist
    if isinstance(data.height, int) and isinstance(data.width, int):
        camera_center = (data.width / 2, data.height / 2)
        max_dist = math.sqrt(camera_center[0]**2 + camera_center[1]**2)
        camera_info.unregister()


def callback(data):
    global count, position, camera_center, max_dist, max_goodness, best_tilt, head_tilt
    image = bridge.imgmsg_to_cv2(data)

    if mode == Model_used.TRACKING:
        result = model.track(image, persist=True, verbose=False,
                             tracker=node_path+"custom_botsort.yaml")
    elif mode == Model_used.DETECTION:
        result = model.predict(image, verbose=False)

    count = len(result[0].boxes)
    avg_position = [0, 0]
    for box, cls in zip(result[0].boxes.xywh.cpu(), result[0].boxes.cls.cpu()):
        if cls == 0:  # Check if ripe
            x, y, w, h = box
            center = getCenter(x, y, w, h)
            avg_position[0] += center[0]
            avg_position[1] += center[1]

    if len(result[0].boxes) > 0:
        avg_position[0] /= count
        avg_position[1] /= count
        history.append({"count": count, "pos": avg_position})

    if len(history) > MAX_HISTORY:
        history.pop(0)

    if len(history) == MAX_HISTORY:
        window_count = 0
        window_pos = [0, 0]
        for elem in history:
            window_count += elem["count"]
            window_pos[0] += elem["pos"][0]
            window_pos[1] += elem["pos"][1]
        window_count /= MAX_HISTORY
        window_count = round(window_count)
        window_pos[0] /= MAX_HISTORY
        window_pos[1] /= MAX_HISTORY
        position_index = max_dist - math.dist(window_pos, camera_center)
        # 10 is a parameter to tune
        goodness_index = position_index + window_count * 5
        if goodness_index >= max_goodness:
            max_goodness = goodness_index
            best_tilt = head_tilt


rospy.init_node("counter_track")
camera_info = rospy.Subscriber("/xtion/rgb/camera_info", CameraInfo, getParams)
rospy.Service("tomato_counting/get_best_tilt", BestPos, giveBestPosition)

rospy.spin()
