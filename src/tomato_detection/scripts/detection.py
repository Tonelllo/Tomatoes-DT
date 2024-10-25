#!/usr/bin/env python
"""Script responsible for publication of the detected tomatoes in pixels."""

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from ultralytics import YOLO
import rospkg
import cv2

"""
Description:
    This script is responsible for the detection of the tomatoes and their
    bounding boxes.
"""

bridge = CvBridge()
rospack = rospkg.RosPack()
node_path = rospack.get_path("tomato_detection") + "/models/"
model = YOLO(node_path + "yolov10m_86p_86r.pt")
pub = rospy.Publisher("tomato_detection/detected_tomatoes",
                      PoseArray, queue_size=5)
imgPublisher = rospy.Publisher(
    "tomato_detection/detection_result", Image, queue_size=1)

print("STARTED")

MAX_HISTORY = 30


def publishTrackingResult(img):
    """
    Publish image with bounding boxes.

    :param sensor_msgs::Image img: Image generated by YOLO
    """
    out_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    imgPublisher.publish(out_img)


def callback(data):
    """
    Process the rgb image given by the robot camera.

    :param sensor_msgs::Image data: Image from the robot
    """
    image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    # result = model.track(image, persist=True, verbose=False,
    #                      tracker=node_path + "custom_botsort.yaml")
    results = model.predict(image, verbose=False, stream=True)

    result = None

    for elem in results:
        result = elem

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
    publishTrackingResult(result.plot())
    # TODO USE HASH TABLE FOR tomatoes
    if result is not None:
        # publishTrackingResult(result.plot())
        # track_ids = result[0].boxes.id.int().cpu().tolist()
        # print(track_ids)
        boxes = result.boxes.xywh.cpu()
        labels = result.boxes.cls.cpu()

        tomatoes = {}
        index = 0
        for box, label in zip(boxes, labels):
            # currently_detected_keys.append(id)
            # if id not in tomatoes:
            # You lose the first
            x, y, w, h = box
            tomatoes[index] = {"pos": (x, y, w, h), "cls": int(label)}
            index += 1

        # This keys are sorted based on how long they have
        # been detected. Smallest indexes are the tomatoes
        # that have not disappeared in the tracking thus
        # they should be quite easily visible
        # sorted_keys_by_time = list(tomatoes.keys())
        # sorted_keys_by_time.sort()
        # print("Sorted by stab: ", sorted_keys_by_time)

        out = PoseArray()
        for key in tomatoes:
            elem = Pose()
            (x, y, w, h) = tomatoes[key]["pos"]
            elem.orientation.x = x
            elem.orientation.y = y
            elem.orientation.z = tomatoes[key]["cls"]
            elem.orientation.w = -1
            elem.position.x = w  # Horizontal diameter in pixels
            elem.position.y = h  # Horizontal diameter in pixels
            out.poses.append(elem)
            # out.header = sensor_msgs.msg.Header()
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
rospy.Subscriber("/tomato_sync/image_rgb", Image, callback)
# model.track(, show=True)

rospy.spin()
