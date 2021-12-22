# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

box = []
cx = []
cy = []
distance = []
bbox_center = []

rospy.sleep(1)

def BB(data):
    global cx,cy, bbox_center
    for box in data.bounding_boxes:
        print('Distance: ', distance)
        rospy.loginfo(
             "BoundingBox_ID: {}, BoundingBox_Class: {}, Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
                box.id, box.Class, box.xmin, box.xmax, box.ymin, box.ymax
            )
        )
    if box.Class == "person":
        cx = (box.xmax + box.xmin) / 2
        print('Center X: ', cx)
        cy = (box.ymax + box.ymin) / 2
        print('Center Y: ',cy)

        bbox_center = Point()
        bbox_center.x = cx
        bbox_center.y = cy
        bbox_center.z = distance
        print('BBox_center: ', bbox_center)
        pub.publish(bbox_center)
    else:
        print("Objekt ist keine Person")

    return cx,cy, bbox_center

def Depth(data):
    global distance
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    depth_array = np.array(depth_image, dtype=np.float32)

    distance = depth_array[cy,cx]
    return distance

rospy.init_node('person_tracking', anonymous=True)
pub = rospy.Publisher('/person_tracking/bbox_center', Point, queue_size=5)

rospy.Publisher('/person_tracking/distance_BB', Point, distance, queue_size=5)
rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, BB, queue_size=1)
rospy.Subscriber('camera/depth/image_rect_raw', Image, Depth, queue_size=1)
rospy.spin()