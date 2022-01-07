#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import sqrt

from yaml import cyaml
import rospy
import actionlib
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Point, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# def callback(msg):
#     print (msg.pose.pose.position)

# rospy.init_node('check_odometry')
# odom_sub = rospy.Subscriber('/odom', Odometry, callback)
# rospy.spin()

# odom_sub --> Nachricht
# x,y des Roboters, bezogen auf das weltkoordinatensystem

# bbox_center = Point()
# bbox_center.x = cx
# bbox_center.y = cy
# bbox_center.z = distance

def calc(data):
    global person_goal
    cx = data.x # Breite der BB in pixel
    cy = data.y # Höhe der BB in pixel
    distance = data.z # Abstand Kamera <--> Person in mm

    # cx = 320        # pixel
    # distance = 1200 # mm

    px_obj_size = distance*1.88*10**-3
    opt_center = 640/2 # Halbe Breite der Auflösung

    if cx < opt_center:
        opt_diff = opt_center - cx
        opt_diff = -opt_diff
        left = True
    else:
        opt_diff = cx - opt_center
        right = True

    print("opt_diff", opt_diff)
    diff = opt_diff*px_obj_size
    print("diff", diff)
    res_dist = sqrt(distance**2 - diff**2)

    person_goal = [diff, res_dist]
    print("person_goal", person_goal)
    return person_goal

def handle_turtle_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "Person"
    t.child_frame_id = 'Turtlebot'
    t.transform.translation.x = msg.pose.pose.position.x + person_goal[0]
    t.transform.translation.y = msg.pose.pose.position.y + person_goal[1]
    t.transform.translation.z = 0.001
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.z)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    print(t)
    br.sendTransform(t)
    pub.publish(t)

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
#     goal.target_pose.pose.position.x = data.transform.translation.x
#     goal.target_pose.pose.position.y = data.transform.translation.y
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
    
if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    # turtlename = rospy.get_param('~turtle')
    # calc()
    rospy.Subscriber('/odom', Odometry, handle_turtle_pose, queue_size=1)
    pub = rospy.Publisher('person_tracking/person', TransformStamped, queue_size=1)
    # rospy.Subscriber('/person_tracking/person', TransformStamped, Person_WP, queue_size=1)
    rospy.Subscriber('/person_tracking/bbox_center', Point, calc, queue_size=1)
    rospy.spin()
