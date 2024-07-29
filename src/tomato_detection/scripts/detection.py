import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from ultralytics import YOLO
import numpy as np

bridge = CvBridge()
model = YOLO("best.pt")
tomatoes = {}
pub = rospy.Publisher("detection/current_best", Pose2D, queue_size=5)
classes = ['RIPE', 'HALF', 'UNRIPE']

MAX_HISTORY = 30


def getCenter(x, y, w, h):
    return ((x+w) / 2, (y+h) / 2)


# def sortByStd(x):
#     print(np.std(tomatoes[x], axis=0))
#     return sum(np.std(tomatoes[x], axis=0))

def callback(data):
    image = bridge.imgmsg_to_cv2(data)
    result = model.track(image, persist=True, verbose=False)
    boxes = result[0].boxes.xywh.cpu()

    annotated = result[0].plot()

    if result[0].boxes.id is not None:
        track_ids = result[0].boxes.id.int().cpu().tolist()
        labels = result[0].boxes.cls.cpu()

        points = []
        currently_detected_keys = []
        for box, id, label in zip(boxes, track_ids, labels):
            if classes[int(label)] == 'RIPE' or classes[int(label)] == 'HALF':
                currently_detected_keys.append(id)
                if id not in tomatoes:
                    # You lose the first
                    tomatoes[id] = []
                else:
                    x, y, w, h = box
                    tomatoes[id].insert(0, getCenter(x, y, w, h))
                    if len(tomatoes[id]) > MAX_HISTORY:
                        tomatoes[id].pop()
                        points.append(np.std(tomatoes[id], axis=0))
                        # print(id, np.std(tomatoes[id], axis=0))

        for key in list(tomatoes):
            if key not in currently_detected_keys:
                tomatoes.pop(key)

        # This keys are sorted based on how long they have
        # been detected. Smallest indexes are the tomatoes
        # that have not disappeared in the tracking thus
        # they should be quite easily visible
        sorted_keys_by_time = list(tomatoes.keys())
        sorted_keys_by_time.sort()
        print("Sorted by stab: ", sorted_keys_by_time)

        if len(tomatoes[sorted_keys_by_time[0]]) == MAX_HISTORY:
            avg = np.mean(tomatoes[sorted_keys_by_time[0]], axis=0)
            out = Pose2D()
            out.x = avg[0]
            out.y = avg[1]
            out.theta = -1
            pub.publish(out)

        # TODO maybe sort by accuracy

        # Too little difference between the values
        # This keys are sorted in increasing stardard deviation
        # value. So stable detections are preferred
        # sorted_keys_by_stability = list(tomatoes.keys())
        # sorted_keys_by_stability.sort(key=sortByStd)
        # print("Sorted by stab: ", sorted_keys_by_stability)

    cv2.imshow("det", annotated)
    cv2.waitKey(1)


rospy.init_node("detector")
rospy.Subscriber("xtion/rgb/image_raw", Image, callback)
# model.track(, show=True)

rospy.spin()
