#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from math import sqrt
from decimal import Decimal, ROUND_HALF_UP
import rospy
import actionlib
import tf_conversions
import tf2_ros
import numpy as np
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from visualization_msgs.msg import Marker

person_goal = []

def camera(data):
    global Fx, Fy, PPx, PPy
    Fx = int(data.K[0])
    PPx = int(data.K[2])
    Fy = int(data.K[4])
    PPy = int(data.K[5])

    # print(Fx, PPx, Fy, PPy)
    return Fx, PPx, Fy, PPy

def calc(bbox_center):
    global person_goal
    angle_per_pixel = 0.0875 # Grad pro Pixel im Bild
    theta = 25
    # if bbox_center.y < 240:
    #     theta = 25 - ((240 - bbox_center.y) * angle_per_pixel)
    # else:
    #     theta = 25 + ((bbox_center.y - 240) * angle_per_pixel)

    # print("theta", theta)
    centerx = int(bbox_center.x)
    centery = int(bbox_center.y)
    centerz = int(bbox_center.z)
    # print("centerz:", centerz)
    # print("Kameradaten", Fx, PPx, Fy, PPy)

    Xtemp = centerz*(centerx - PPx)/Fx
    Ytemp = centerz*(centery - PPy)/Fy
    Ztemp = centerz

    Xtarget = Xtemp - 35
    Ytarget = -(Ztemp*math.sin(theta) + Ytemp*math.cos(theta))
    Ztarget = Ztemp*math.cos(theta) #+ Ytemp*math.sin(theta)

    # if theta > 0:
    #     res_dist = sqrt(Ztarget**2 - Ytarget**2)
    #     print("Winkelfunktion: ", res_dist)
    # else:
    #     pass

    # print("Abstand Realsense: ", bbox_center.z)
    # print("Abstand Berechnung mit theta: ", Ztarget, Xtarget) # in mm
    person_goal = [Ztarget, Xtarget]
    person_goal = Point()
    person_goal.x = Ztarget
    person_goal.y = Xtarget
    person_goal.z = 0.001
    # print("person_goal", person_goal)
    pub_goal.publish(person_goal)

    # cx = data.x # Breite der BB in pixel
    # cy = data.y # Höhe der BB in pixel
    # distance = data.z # Abstand Kamera <--> Person in mm

    # # cx = 320        # pixel
    # # distance = 1200 # mm

    # px_obj_size = distance*1.88*10**-3
    # opt_center = 640/2 # Halbe Breite der Auflösung

    return person_goal

def Person_Marker(data):
    publisher = rospy.Publisher('person_tracking/marker', Marker, queue_size=1)
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.id = 0
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = data.x/1000
    marker.pose.position.y = -data.y/1000
    marker.pose.position.z = data.z

    publisher.publish(marker)
    rospy.sleep(1)

    return marker

def Person_WP(marker):
    print("Starte Navigation")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.get_state()
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()

    # define loop
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = marker.pose.position.x
    goal.target_pose.pose.position.y = marker.pose.position.y
    goal.target_pose.pose.position.z = marker.pose.position.z
    goal.target_pose.pose.orientation.w = 0.09012465928 # fixed angles
    goal.target_pose.pose.orientation.z = 0.098917586 # of robot position for all goals
    print("Aktuelles Ziel x:", goal.target_pose.pose.position.x, "y:", goal.target_pose.pose.position.y)

    # client.cancel_all_goals()
    client.send_goal(goal)
    
    # wait = client.wait_for_result()
    # if not wait:
    #         rospy.logerr("Action server DOWN ;/ ")
    #         return False
    # else:
    #         print("A Goal is Executed") 
#     return True

rospy.init_node('RVIZ_Person')
rospy.Subscriber('camera/color/camera_info', CameraInfo, camera, queue_size=1)
rospy.Subscriber('/person_tracking/bbox_center', Point, calc, queue_size=1)
rospy.Subscriber('/person_tracking/goal', Point, Person_Marker, queue_size=1)
rospy.Subscriber('/person_tracking/marker', Marker, Person_WP, queue_size=1)
pub = rospy.Publisher('person_tracking/person', TransformStamped, queue_size=1)
pub_goal = rospy.Publisher('person_tracking/goal', Point, queue_size=1)

rospy.spin()