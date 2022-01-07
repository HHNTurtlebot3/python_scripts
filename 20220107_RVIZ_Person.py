#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from math import sqrt
from yaml import cyaml
from decimal import Decimal, ROUND_HALF_UP
import rospy
import actionlib
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Point, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# def callback(msg):
#     print (msg.pose.pose.position)

# rospy.init_node('check_odometry')
# odom_sub = rospy.Subscriber('/odom', Odometry, callback)

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
    theta = 25
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
    Ztarget = Ztemp*math.cos(theta) + Ytemp*math.sin(theta)

    res_dist = sqrt(Ztarget**2 - Ytarget**2)

    print("Abstand Realsense: ", bbox_center.z)
    print("Abstand Berechnung mit theta: ", Ztarget, Xtarget) # in mm
    print("Winkelfunktion: ", res_dist)
    # coordinates_text = "(" + str(Decimal(str(Xtarget)).quantize(Decimal('0'), rounding=ROUND_HALF_UP)) + \
    #                     ", " + str(Decimal(str(Ytarget)).quantize(Decimal('0'), rounding=ROUND_HALF_UP)) + \
    #                     ", " + str(Decimal(str(Ztarget)).quantize(Decimal('0'), rounding=ROUND_HALF_UP)) + ")"

    # cx = data.x # Breite der BB in pixel
    # cy = data.y # Höhe der BB in pixel
    # distance = data.z # Abstand Kamera <--> Person in mm

    # # cx = 320        # pixel
    # # distance = 1200 # mm

    # px_obj_size = distance*1.88*10**-3
    # opt_center = 640/2 # Halbe Breite der Auflösung

    # if cx < opt_center:
    #     opt_diff = opt_center - cx
    #     opt_diff = -opt_diff
    #     left = True
    # else:
    #     opt_diff = cx - opt_center
    #     right = True

    # print("opt_diff", opt_diff)
    # diff = opt_diff*px_obj_size
    # print("diff", diff)
    # res_dist = sqrt(distance**2 - diff**2)

    # person_goal = [diff, res_dist]

    # person_goal = Point()
    # person_goal.x = diff
    # person_goal.y = 0.001
    # person_goal.z = res_dist
    # print("person_goal", person_goal)
    # pub_goal.publish(person_goal)
    return person_goal

def handle_turtle_pose(odom):
    # print(msg)

    if not person_goal:
        pass
        # print("Keine Person gefunden")
    else:
        pass
        # br = tf2_ros.TransformBroadcaster()
        # t = geometry_msgs.msg.TransformStamped()

        # person_goal_ = rospy.Subscriber('person_tracking/goal', Point, queue_size=1)
        # t.header.stamp = rospy.Time.now()
        # t.header.frame_id = "Person"
        # t.child_frame_id = 'Turtlebot'

        # t.transform.translation.x = odom.pose.pose.position.x + person_goal[0]
        # t.transform.translation.y = odom.pose.pose.position.y + person_goal[1]    
        # t.transform.translation.x = odom.pose.pose.position.x + goal.x
        # t.transform.translation.y = odom.pose.pose.position.y + goal.y
        # t.transform.translation.z = 0.001
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, odom.pose.orientation.z)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # print(t)
        # br.sendTransform(t)
        # pub.publish(t)

# def Person_WP(data):
#     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     client.wait_for_server()

# #     TargetList = []
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()

#     # define loop
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = data.transform.translation.x --> unser Z Wert, Abstand Person zum Weltkoordinantensystem
#     goal.target_pose.pose.position.y = data.transform.translation.y --> unser X
#     goal.target_pose.pose.position.z = -0.001 # fixed z coordinate
#     goal.target_pose.pose.orientation.w = 0.09012465928 # fixed angles
#     goal.target_pose.pose.orientation.z = 0.098917586 # of robot position for all goals
# #         print("Aktuelles Ziel x:", goal.target_pose.pose.position.x, "y:", goal.target_pose.pose.position.y)

#     client.send_goal(goal)
#     wait = client.wait_for_result()
#         if not wait:
#             rospy.logerr("Action server DOWN ;/ ")
#             return False
#         else:
#             print("A Goal is Executed") 
#     return True
    
# if __name__ == '__main__':
rospy.init_node('tf2_turtle_broadcaster')
    # turtlename = rospy.get_param('~turtle')
    # calc()
rospy.Subscriber('camera/color/camera_info', CameraInfo, camera, queue_size=1)
rospy.Subscriber('/person_tracking/bbox_center', Point, calc, queue_size=1)
rospy.Subscriber('/odom', Odometry, handle_turtle_pose, queue_size=1)
# rospy.Subscriber('person_tracking/goal', Point, handle_turtle_pose, queue_size=1)
pub = rospy.Publisher('person_tracking/person', TransformStamped, queue_size=1)
pub_goal = rospy.Publisher('person_tracking/goal', Point, queue_size=1)
    # rospy.Subscriber('/person_tracking/person', TransformStamped, Person_WP, queue_size=1)
rospy.spin()