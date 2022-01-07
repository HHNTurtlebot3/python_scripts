# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

box = []
cx = []
cy = []
distance = []
bbox_center = []
x_range = []
y_range = []
px = []
py = []

rospy.sleep(1)

def pixel(data):
    global px, py, px_alt, py_alt
    px = int(data.point.x)
    py = int(data.point.y)

    return px, py

def BB(data):
    global cx,cy, bbox_center, x_range, y_range 
    for box in data.bounding_boxes:
        pass
        # rospy.loginfo(
        #      "BoundingBox_ID: {}, BoundingBox_Class: {}, Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
        #         box.id, box.Class, box.xmin, box.xmax, box.ymin, box.ymax
        #     )
        # )

    # print("PIXEL!!!", px, py)
    xmin_ = box.xmin + 40
    xmax_ = box.xmax - 40
    ymin_ = box.ymin + 40
    ymax_ = box.ymax - 40

    if distance == 0: # and box.probability > 0.6: # zusätzlich noch die probability prüfen, Schwellwert > 0.60 --> box.probability >0.60
        print('Abstand ist 0!')
    else:
        # print('Distance: ', distance)
        if box.Class == "person": # and px in range(xmin_, xmax_) and py in range(ymin_, ymax_):
            cx = (box.xmax + box.xmin) / 2
            print('Center X: ', cx)
            cy = (box.ymax + box.ymin) / 2
            print('Center Y: ',cy)

            bbox_center = Point()
            bbox_center.x = cx
            bbox_center.y = cy
            bbox_center.z = distance
            print('BBox_center: ', bbox_center)
            print("Info wird gesendet")
            pub.publish(bbox_center)
        else:
            print("Bedingung nicht erfüllt!")

    return cx,cy, bbox_center

def Depth(data):
    global distance
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    depth_array = np.array(depth_image, dtype=np.float32)

    distance = depth_array[cy,cx]
    return distance

def Color(data):
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    color_array = np.array(color_image, dtype=np.float32)
    pub_color.publish(color_array)
    return color_array

rospy.init_node('person_tracking', anonymous=True)

pub = rospy.Publisher('/person_tracking/bbox_center', Point, queue_size=5)
marker_pub = rospy.Publisher('/person_tracking/rviz', Marker, queue_size=5)
pub_color = rospy.Publisher('/person_tracking/color_frame', numpy_msg(Floats), queue_size=1)
rospy.Publisher('/person_tracking/distance_BB', Point, distance, queue_size=5)

rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, BB, queue_size=1)
rospy.Subscriber('camera/depth/image_rect_raw', Image, Depth, queue_size=1)
rospy.Subscriber('camera/color/image_raw', Image, Color, queue_size=1)
rospy.Subscriber('aruco_single/pixel', PointStamped, pixel, queue_size=1)
rospy.spin()