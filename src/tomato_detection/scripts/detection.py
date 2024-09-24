#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np
import rospkg
import copy
import cv2

"""
Description:
    This script is responsible for the detection of the tomatoes and their
    bounding boxes.
"""

bridge = CvBridge()
rospack = rospkg.RosPack()
node_path = rospack.get_path("tomato_detection") + "/models/"
model = YOLO(node_path + "tomato_80prec_70rec.pt")
tomatoes = {}
pub = rospy.Publisher("tomato_detection/detected_tomatoes",
                      PoseArray, queue_size=5)
imgPublisher = rospy.Publisher(
    "tomato_detection/detection_result", Image, queue_size=1)

print("STARTED")

MAX_HISTORY = 30


def publishTrackingResult(img):
    out_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    imgPublisher.publish(out_img)


def callback(data):
    image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    result = model.track(image, persist=True, verbose=False,
                         tracker=node_path + "custom_botsort.yaml")
    boxes = result[0].boxes.xywh.cpu()

    """
    0 -> 'fully_ripened'
    1 -> 'half_ripened'
    2 -> 'green'
    """

    # wanted_result = copy.deepcopy(result[0])
    # wanted_result.boxes = [
    #     box for box in result[0].boxes if box.cls in [0, 1]]

    # out = []
    # for box in wanted_result.boxes:
    #     out.append(box.cls)
    # print(out)

    # publishTrackingResult(wanted_result.plot())
    # TODO Why does it keep displaying all the bounding boxes?
    publishTrackingResult(result[0].plot())

    # TODO USE HASH TABLE FOR tomatoes
    if result[0].boxes.id is not None:
        publishTrackingResult(result[0].plot())
        track_ids = result[0].boxes.id.int().cpu().tolist()
        labels = result[0].boxes.cls.cpu()

        points = []
        currently_detected_keys = []
        for box, id, label in zip(boxes, track_ids, labels):
            currently_detected_keys.append(id)
            if id not in tomatoes:
                # You lose the first
                tomatoes[id] = {"arr": [], "cls": int(label), "id": int(id)}
            else:
                x, y, w, h = box
                tomatoes[id]["arr"].insert(0, (x, y, w))
                if len(tomatoes[id]["arr"]) > MAX_HISTORY:
                    tomatoes[id]["arr"].pop()
                    points.append(np.std(tomatoes[id]["arr"], axis=0))

        for key in list(tomatoes):
            if key not in currently_detected_keys:
                tomatoes.pop(key)
        # This keys are sorted based on how long they have
        # been detected. Smallest indexes are the tomatoes
        # that have not disappeared in the tracking thus
        # they should be quite easily visible
        sorted_keys_by_time = list(tomatoes.keys())
        sorted_keys_by_time.sort()
        # print("Sorted by stab: ", sorted_keys_by_time)

        out = PoseArray()
        for tomato in sorted_keys_by_time:
            if len(tomatoes[tomato]["arr"]) == MAX_HISTORY:
                elem = Pose()
                avg = np.mean(tomatoes[tomato]["arr"], axis=0)
                elem.orientation.x = avg[0]
                elem.orientation.y = avg[1]
                elem.orientation.z = tomatoes[tomato]["cls"]
                elem.orientation.w = tomatoes[tomato]["id"]
                elem.position.x = avg[2]  # Horizontal radius in pixels
                out.poses.append(elem)
                # out.header = sensor_msgs.msg.Header()
                # out.header.stamp = rospy.Time.now()
                out.header.stamp = data.header.stamp
        pub.publish(out)

        # TODO maybe sort by accuracy

        # Too little difference between the values
        # This keys are sorted in increasing stardard deviation
        # value. So stable detections are preferred
        # sorted_keys_by_stability = list(tomatoes.keys())
        # sorted_keys_by_stability.sort(key=sortByStd)
        # print("Sorted by stab: ", sorted_keys_by_stability)


rospy.init_node("detector")
# rospy.Subscriber("xtion/rgb/image_rect_color", Image, callback)
rospy.Subscriber("xtion/rgb/image_raw", Image, callback)
# model.track(, show=True)

rospy.spin()
